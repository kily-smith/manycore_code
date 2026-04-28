#include "GM6020.h"

#include "bsp_can.h"
#include "pid.h"

extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern uint16_t can_cnt;

static pid_struct_t g_gm6020_pid;
static pid_struct_t g_gm6020_pos_pid;
static int16_t g_gm6020_target_speed = 0;
static int16_t g_gm6020_target_pos_raw = 0;
static uint8_t g_gm6020_running = 0U;
static uint8_t g_gm6020_pos_mode = 0U;

static const int16_t g_gm6020_speed_step = 60;
static const int16_t g_gm6020_speed_max = 180;
static const GPIO_PinState g_key_active_level = GPIO_PIN_SET;
static const uint32_t g_key_led_flash_ms = 80U;
static const uint32_t g_key_debounce_ms = 20U;
static const uint32_t g_control_period_ms = 2U;
static const int16_t g_gm6020_pos_speed_limit = 1000;
static const int16_t g_gm6020_angle_raw_max = 8191;
static const int16_t g_gm6020_angle_wrap = 8192;

static uint8_t g_key_inited = 0U;
static GPIO_PinState g_key_last_sample = GPIO_PIN_SET;
static GPIO_PinState g_key_stable = GPIO_PIN_SET;
static uint32_t g_key_change_tick = 0U;
static uint32_t g_key_led_off_tick = 0U;
static uint32_t g_control_last_tick = 0U;
static uint16_t g_last_can_cnt = 0U;
static uint8_t g_has_feedback = 0U;

static int16_t GM6020_DegToRaw(int16_t deg)
{
  /* UI uses [-180, 180] deg; map to [0, 360) to keep direction valid. */
  int32_t d = (int32_t)deg % 360;
  if (d < 0)
  {
    d += 360;
  }
  return (int16_t)((d * g_gm6020_angle_wrap) / 360);
}

static int16_t GM6020_WrapAngleError(int16_t target, int16_t current)
{
  int32_t err = (int32_t)target - (int32_t)current;
  while (err > (g_gm6020_angle_wrap / 2))
  {
    err -= g_gm6020_angle_wrap;
  }
  while (err < -(g_gm6020_angle_wrap / 2))
  {
    err += g_gm6020_angle_wrap;
  }
  return (int16_t)err;
}

void GM6020_UIStart(UI_MotorId_t motor, int16_t target_speed)
{
  if (motor != UI_MOTOR_GM6020)
  {
    return;
  }

  g_gm6020_target_speed = target_speed;
  g_gm6020_running = 1U;
  g_gm6020_pos_mode = 0U;
}

void GM6020_UISetSpeed(UI_MotorId_t motor, int16_t target_speed)
{
  if (motor != UI_MOTOR_GM6020)
  {
    return;
  }

  g_gm6020_target_speed = target_speed;
  g_gm6020_pos_mode = 0U;
}

void GM6020_UIStartPos(UI_MotorId_t motor, int16_t target_pos_deg)
{
  if (motor != UI_MOTOR_GM6020)
  {
    return;
  }

  g_gm6020_target_pos_raw = GM6020_DegToRaw(target_pos_deg);
  g_gm6020_running = 1U;
  g_gm6020_pos_mode = 1U;
}

void GM6020_UISetPos(UI_MotorId_t motor, int16_t target_pos_deg)
{
  if (motor != UI_MOTOR_GM6020)
  {
    return;
  }

  g_gm6020_target_pos_raw = GM6020_DegToRaw(target_pos_deg);
  g_gm6020_pos_mode = 1U;
}

void GM6020_UIStop(UI_MotorId_t motor)
{
  if (motor != UI_MOTOR_GM6020)
  {
    return;
  }

  g_gm6020_running = 0U;
  g_gm6020_target_speed = 0;
  g_gm6020_target_pos_raw = 0;
  g_gm6020_pid.i_out = 0.0f;
  g_gm6020_pid.output = 0.0f;
  g_gm6020_pos_pid.i_out = 0.0f;
  g_gm6020_pos_pid.output = 0.0f;
  set_motor_voltage(0, 0, 0, 0, 0);
}

void GM6020_Init(void)
{
  pid_init(&g_gm6020_pid, 40.0f, 3.0f, 0.0f, 30000.0f, 30000.0f);
  pid_init(&g_gm6020_pos_pid, 1.2f, 0.0f, 0.0f, (float)g_gm6020_pos_speed_limit, (float)g_gm6020_pos_speed_limit);
  g_gm6020_target_speed = 0;
  g_gm6020_target_pos_raw = 0;
  g_gm6020_running = 0U;
  g_gm6020_pos_mode = 0U;
  g_key_inited = 0U;
  g_key_led_off_tick = 0U;
  g_control_last_tick = 0U;
  g_last_can_cnt = can_cnt;
  g_has_feedback = 0U;
}

void GM6020_Task(uint32_t now_ms, GPIO_PinState key_sample)
{
  uint8_t key_press_event = 0U;

  if (!g_key_inited)
  {
    g_key_inited = 1U;
    g_key_last_sample = key_sample;
    g_key_stable = key_sample;
    g_key_change_tick = now_ms;
  }

  if (key_sample != g_key_last_sample)
  {
    g_key_last_sample = key_sample;
    g_key_change_tick = now_ms;
  }

  if (((now_ms - g_key_change_tick) >= g_key_debounce_ms) && (key_sample != g_key_stable))
  {
    g_key_stable = key_sample;
    if (g_key_stable == g_key_active_level)
    {
      key_press_event = 1U;
    }
  }

  if (key_press_event)
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    g_key_led_off_tick = now_ms + g_key_led_flash_ms;

    g_gm6020_target_speed += g_gm6020_speed_step;
    if (g_gm6020_target_speed > g_gm6020_speed_max)
    {
      GM6020_UIStop(UI_MOTOR_GM6020);
    }
    else if (g_gm6020_running)
    {
      GM6020_UISetSpeed(UI_MOTOR_GM6020, g_gm6020_target_speed);
    }
    else
    {
      GM6020_UIStart(UI_MOTOR_GM6020, g_gm6020_target_speed);
    }
  }
  else if ((g_key_led_off_tick != 0U) && ((int32_t)(now_ms - g_key_led_off_tick) >= 0))
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    g_key_led_off_tick = 0U;
  }

  if ((now_ms - g_control_last_tick) >= g_control_period_ms)
  {
    int16_t voltage_cmd = 0;

    g_control_last_tick = now_ms;
    if (g_gm6020_running)
    {
      int16_t speed_ref = g_gm6020_target_speed;
      if (g_gm6020_pos_mode)
      {
        int16_t current_raw = motor_info[0].rotor_angle;
        if (current_raw < 0)
        {
          current_raw = 0;
        }
        if (current_raw > g_gm6020_angle_raw_max)
        {
          current_raw = g_gm6020_angle_raw_max;
        }
        speed_ref = (int16_t)pid_calc(&g_gm6020_pos_pid, (float)GM6020_WrapAngleError(g_gm6020_target_pos_raw, current_raw), 0.0f);
      }
      float out = pid_calc(&g_gm6020_pid, (float)speed_ref, (float)motor_info[0].rotor_speed);
      voltage_cmd = (int16_t)out;
    }
    else
    {
      g_gm6020_pid.i_out = 0.0f;
      g_gm6020_pos_pid.i_out = 0.0f;
    }

    set_motor_voltage(0, voltage_cmd, 0, 0, 0);
  }

  if (can_cnt != g_last_can_cnt)
  {
    g_last_can_cnt = can_cnt;
    g_has_feedback = 1U;
  }
}

uint8_t GM6020_HasFeedback(void)
{
  return g_has_feedback;
}

int16_t GM6020_GetTargetSpeed(void)
{
  return g_gm6020_target_speed;
}

uint8_t GM6020_IsRunning(void)
{
  return g_gm6020_running;
}
