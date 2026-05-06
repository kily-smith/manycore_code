#include "emm_motor.h"

#include "Emm_V5.h"
#include "main.h"

/* Tunables copied from Bollard_control baseline. */
#define EMM_QUERY_PERIOD_MS            80U
#define EMM_RESPONSE_GAP_MS            5U
#define EMM_RESPONSE_TIMEOUT_MS        50U
#define EMM_BOOT_DELAY_MS              500U

#define EMM_POS_CMD_VEL_RPM            1000U
#define EMM_POS_CMD_ACC                0U

#define EMM_HEIGHT_MAX_MM              400
#define EMM_RUN_HEIGHT_MAX_MM          300
#define EMM_HEIGHT_MIN_MM              0

#define EMM_PULSE_PER_MM_CMD           800UL /* 3200 / 4 */
#define EMM_PULSE_PER_REV              65536.0f

typedef enum
{
  EMM_QUERY_TPOS = 0,
  EMM_QUERY_VEL,
  EMM_QUERY_CPOS
} EmmQuery_t;

static UART_HandleTypeDef *g_uart = &huart2;
static uint8_t g_addr = 1U;

static uint8_t g_rx_byte = 0U;
static uint8_t g_rx_buf[32];
static volatile uint16_t g_rx_len = 0U;
static uint32_t g_last_rx_tick_ms = 0U;
static uint32_t g_last_query_tick_ms = 0U;
static uint32_t g_wait_start_ms = 0U;
static uint32_t g_boot_tick_ms = 0U;
static uint8_t g_ready = 0U;
static uint8_t g_waiting_response = 0U;
static EmmQuery_t g_query = EMM_QUERY_TPOS;

static float g_target_pos_deg = 0.0f;
static float g_current_pos_deg = 0.0f;
static float g_speed_rpm = 0.0f;
static int32_t g_height_setting_mm = 0;
static uint8_t g_dir_up = 1U;

static void Emm_StartReceiveIT(void)
{
  (void)HAL_UART_Receive_IT(g_uart, &g_rx_byte, 1U);
}

static float Emm_PosRawToDeg(uint32_t pos_raw, uint8_t sign)
{
  float deg = ((float)pos_raw * 360.0f) / EMM_PULSE_PER_REV;
  return (sign != 0U) ? -deg : deg;
}

static void Emm_ProcessFrame(const uint8_t *buf, uint16_t len)
{
  uint32_t value = 0U;

  if ((buf == 0) || (len < 2U) || (buf[0] != g_addr))
  {
    return;
  }

  switch (buf[1])
  {
    case 0x33: /* TPOS */
      if (len == 8U)
      {
        value = ((uint32_t)buf[3] << 24)
              | ((uint32_t)buf[4] << 16)
              | ((uint32_t)buf[5] << 8)
              | ((uint32_t)buf[6] << 0);
        g_target_pos_deg = Emm_PosRawToDeg(value, buf[2]);
      }
      break;

    case 0x35: /* VEL */
      if (len == 6U)
      {
        value = ((uint16_t)buf[3] << 8) | ((uint16_t)buf[4] << 0);
        g_speed_rpm = (float)value;
        if (buf[2] != 0U)
        {
          g_speed_rpm = -g_speed_rpm;
        }
      }
      break;

    case 0x36: /* CPOS */
      if (len == 8U)
      {
        value = ((uint32_t)buf[3] << 24)
              | ((uint32_t)buf[4] << 16)
              | ((uint32_t)buf[5] << 8)
              | ((uint32_t)buf[6] << 0);
        g_current_pos_deg = Emm_PosRawToDeg(value, buf[2]);
      }
      break;

    default:
      break;
  }
}

static void Emm_SendNextQuery(void)
{
  switch (g_query)
  {
    case EMM_QUERY_TPOS:
      Emm_V5_Read_Sys_Params(g_addr, S_TPOS);
      g_query = EMM_QUERY_VEL;
      break;
    case EMM_QUERY_VEL:
      Emm_V5_Read_Sys_Params(g_addr, S_VEL);
      g_query = EMM_QUERY_CPOS;
      break;
    case EMM_QUERY_CPOS:
    default:
      Emm_V5_Read_Sys_Params(g_addr, S_CPOS);
      g_query = EMM_QUERY_TPOS;
      break;
  }
  g_waiting_response = 1U;
}

static uint8_t Emm_IsMovingUp(uint8_t dir_cmd)
{
  /* Vendor dir: 1=CCW,0=CW. We map "UP" by `g_dir_up`. */
  uint8_t up_is_ccw = (g_dir_up != 0U) ? 1U : 0U;
  return (dir_cmd == up_is_ccw) ? 1U : 0U;
}

void EmmMotor_Init(UART_HandleTypeDef *huart, uint8_t addr)
{
  if (huart != 0)
  {
    g_uart = huart;
  }
  g_addr = (addr == 0U) ? 1U : addr;

  Emm_V5_SetUart(g_uart);

  g_rx_len = 0U;
  g_last_rx_tick_ms = 0U;
  g_last_query_tick_ms = 0U;
  g_wait_start_ms = 0U;
  g_boot_tick_ms = HAL_GetTick();
  g_ready = 0U;
  g_waiting_response = 0U;
  g_query = EMM_QUERY_TPOS;

  g_target_pos_deg = 0.0f;
  g_current_pos_deg = 0.0f;
  g_speed_rpm = 0.0f;
  g_height_setting_mm = 0;
  g_dir_up = 1U;

  Emm_StartReceiveIT();
}

void EmmMotor_Task(uint32_t now_ms)
{
  if ((g_ready == 0U) && ((now_ms - g_boot_tick_ms) >= EMM_BOOT_DELAY_MS))
  {
    g_ready = 1U;
  }

  if ((g_rx_len > 0U) && ((now_ms - g_last_rx_tick_ms) >= EMM_RESPONSE_GAP_MS))
  {
    Emm_ProcessFrame(g_rx_buf, g_rx_len);
    g_rx_len = 0U;
    g_waiting_response = 0U;
  }

  if ((g_waiting_response != 0U) && ((now_ms - g_wait_start_ms) >= EMM_RESPONSE_TIMEOUT_MS))
  {
    g_waiting_response = 0U;
    g_rx_len = 0U;
  }

  if ((g_ready != 0U)
   && (g_waiting_response == 0U)
   && ((now_ms - g_last_query_tick_ms) >= EMM_QUERY_PERIOD_MS))
  {
    Emm_SendNextQuery();
    g_last_query_tick_ms = now_ms;
    g_wait_start_ms = now_ms;
  }
}

void EmmMotor_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != g_uart)
  {
    return;
  }

  if (g_rx_len < (uint16_t)sizeof(g_rx_buf))
  {
    g_rx_buf[g_rx_len++] = g_rx_byte;
  }
  else
  {
    g_rx_len = 0U;
  }
  g_last_rx_tick_ms = HAL_GetTick();
  Emm_StartReceiveIT();
}

void EmmMotor_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != g_uart)
  {
    return;
  }
  g_rx_len = 0U;
  Emm_StartReceiveIT();
}

float EmmMotor_GetCurrentPosDeg(void)
{
  return g_current_pos_deg;
}

float EmmMotor_GetTargetPosDeg(void)
{
  return g_target_pos_deg;
}

float EmmMotor_GetSpeedRpm(void)
{
  return g_speed_rpm;
}

void EmmMotor_SetHeightSettingMm(int32_t height_mm, uint8_t dir_up)
{
  if (height_mm < EMM_HEIGHT_MIN_MM) height_mm = EMM_HEIGHT_MIN_MM;
  if (height_mm > EMM_HEIGHT_MAX_MM) height_mm = EMM_HEIGHT_MAX_MM;

  g_height_setting_mm = height_mm;
  g_dir_up = (dir_up != 0U) ? 1U : 0U;
}

int32_t EmmMotor_GetHeightSettingMm(void)
{
  return g_height_setting_mm;
}

uint8_t EmmMotor_GetDirUp(void)
{
  return g_dir_up;
}

static float Emm_HeightFromPosDeg(float pos_deg)
{
  float deg = (pos_deg < 0.0f) ? -pos_deg : pos_deg;
  return deg / 90.0f; /* 1mm = 90deg (4mm per rev) */
}

void EmmMotor_StartMoveToSetting(void)
{
  int32_t run_height_mm = g_height_setting_mm;
  float cur_height_mm = Emm_HeightFromPosDeg(g_current_pos_deg);
  uint8_t dir = 0U;
  uint32_t pulse = 0U;

  if (run_height_mm < EMM_HEIGHT_MIN_MM) run_height_mm = EMM_HEIGHT_MIN_MM;
  if (run_height_mm > EMM_RUN_HEIGHT_MAX_MM) run_height_mm = EMM_RUN_HEIGHT_MAX_MM;

  /* Decide direction based on target vs current height. */
  {
    uint8_t want_up = (run_height_mm >= (int32_t)(cur_height_mm + 0.01f)) ? 1U : 0U;
    uint8_t up_is_ccw = (g_dir_up != 0U) ? 1U : 0U;
    dir = (want_up != 0U) ? up_is_ccw : (uint8_t)(1U - up_is_ccw);
  }

#ifdef EMM_LIMIT_UP_GPIO_Port
  if ((Emm_IsMovingUp(dir) != 0U) && (EmmMotor_IsUpperLimitActive() != 0U))
  {
    EmmMotor_StopMove();
    return;
  }
#endif
#ifdef EMM_LIMIT_DOWN_GPIO_Port
  if ((Emm_IsMovingUp(dir) == 0U) && (EmmMotor_IsLowerLimitActive() != 0U))
  {
    EmmMotor_StopMove();
    return;
  }
#endif

  pulse = (uint32_t)run_height_mm * EMM_PULSE_PER_MM_CMD;
  Emm_V5_Pos_Control(g_addr, dir, EMM_POS_CMD_VEL_RPM, EMM_POS_CMD_ACC, pulse, 1U, 0U);
  g_target_pos_deg = (float)run_height_mm * 90.0f;
}

void EmmMotor_StopMove(void)
{
  Emm_V5_Stop_Now(g_addr, 0U);
}

#ifdef EMM_LIMIT_UP_GPIO_Port
uint8_t EmmMotor_IsUpperLimitActive(void)
{
  return (HAL_GPIO_ReadPin(EMM_LIMIT_UP_GPIO_Port, EMM_LIMIT_UP_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}
#endif

#ifdef EMM_LIMIT_DOWN_GPIO_Port
uint8_t EmmMotor_IsLowerLimitActive(void)
{
  return (HAL_GPIO_ReadPin(EMM_LIMIT_DOWN_GPIO_Port, EMM_LIMIT_DOWN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}
#endif

