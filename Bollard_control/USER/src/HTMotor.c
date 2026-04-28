#include "HTMotor.h"

#include "can.h"
#include "main.h"

/* HT-03 MIT ranges from vendor demo */
static const float k_p_min = -95.5f;
static const float k_p_max = 95.5f;
static const float k_v_min = -45.0f;
static const float k_v_max = 45.0f;
static const float k_kp_min = 0.0f;
static const float k_kp_max = 500.0f;
static const float k_kd_min = 0.0f;
static const float k_kd_max = 5.0f;
static const float k_t_min = -18.0f;
static const float k_t_max = 18.0f;

#define HT_SPEED_KP_DEFAULT 0.0f
#define HT_SPEED_KD_DEFAULT 1.0f 
#define HT_SPEED_TFF_DEFAULT 0.8f
#define HT_POS_KP_DEFAULT   60.0f
#define HT_POS_KD_DEFAULT   1.5f
#define HT_POS_TFF_DEFAULT  0.0f
#define HT_CTRL_PERIOD_MS   2U
#define HT_POS_SLEW_RAD_PER_TICK  0.10f
#define HT_VEL_SLEW_RAD_PER_TICK  0.25f
#define HT_TFF_SLEW_PER_TICK      0.08f

static uint8_t g_enabled = 0U;
static float g_target_pos = 0.0f;
static float g_target_vel = 0.0f;
static float g_target_tff = 0.0f;
static float g_cmd_pos = 0.0f;
static float g_cmd_vel = 0.0f;
static float g_cmd_tff = 0.0f;
static float g_kp = 5.0f;
static float g_kd = 0.2f;
static uint32_t g_last_tx_ms = 0U;
static uint32_t g_soft_start_until_ms = 0U;

static HTMotor_Feedback_t g_fb = {0};

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* UI target is output-shaft unit; convert to motor-side unit for MIT command. */
static float HT_OutputSpeedToMotorSpeed(float out_rad_s)
{
    return out_rad_s * HT_MOTOR_GEAR_RATIO;
}

static float HT_OutputRpmToMotorRadPerSec(float out_rpm)
{
    /* rpm -> rad/s -> motor-side rad/s */
    float out_rad_s = out_rpm * 0.10471976f; /* 2*pi/60 */
    return HT_OutputSpeedToMotorSpeed(out_rad_s);
}

static float HT_OutputDegToMotorRad(float out_deg)
{
    return (out_deg * 3.1415926f / 180.0f) * HT_MOTOR_GEAR_RATIO;
}

static float HT_RawPosToMotorRad(uint16_t raw16)
{
    return ((float)raw16 * 191.0f / 65535.0f) - 95.5f;
}

static float HT_SlewStep(float cur, float target, float step)
{
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_int = (1UL << bits) - 1UL;
    if (offset <= 0.0f) return 0U;
    if (offset >= span) return max_int;
    return (uint32_t)((offset * (float)max_int) / span);
}

static void ht_send_raw(uint16_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mb = 0U;
    tx.StdId = std_id;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mb);
}

static void ht_send_cmd(float p, float v, float kp, float kd, float tff)
{
    uint32_t p_u = float_to_uint(p,  k_p_min,  k_p_max,  16);
    uint32_t v_u = float_to_uint(v,  k_v_min,  k_v_max,  12);
    uint32_t kp_u= float_to_uint(kp, k_kp_min, k_kp_max, 12);
    uint32_t kd_u= float_to_uint(kd, k_kd_min, k_kd_max, 12);
    uint32_t t_u = float_to_uint(tff,k_t_min,  k_t_max,  12);

    uint8_t d[8];
    d[0] = (uint8_t)(p_u >> 8);
    d[1] = (uint8_t)(p_u & 0xFF);
    d[2] = (uint8_t)(v_u >> 4);
    d[3] = (uint8_t)(((v_u & 0xF) << 4) | (kp_u >> 8));
    d[4] = (uint8_t)(kp_u & 0xFF);
    d[5] = (uint8_t)(kd_u >> 4);
    d[6] = (uint8_t)(((kd_u & 0xF) << 4) | (t_u >> 8));
    d[7] = (uint8_t)(t_u & 0xFF);

    ht_send_raw((uint16_t)HT_MOTOR_CAN_ID, d);
}

static void ht_send_special(uint8_t b)
{
    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,b};
    ht_send_raw((uint16_t)HT_MOTOR_CAN_ID, d);
}

void HTMotor_Init(void)
{
    g_enabled = 0U;
    g_target_pos = 0.0f;
    g_target_vel = 0.0f;
    g_target_tff = 0.0f;
    g_cmd_pos = 0.0f;
    g_cmd_vel = 0.0f;
    g_cmd_tff = 0.0f;
    g_kp = HT_POS_KP_DEFAULT;
    g_kd = HT_POS_KD_DEFAULT;
    g_last_tx_ms = 0U;
    g_soft_start_until_ms = 0U;
    g_fb.last_rx_ms = 0U;
}

void HTMotor_Task(uint32_t now_ms)
{
    (void)now_ms;
    if (!g_enabled)
    {
        return;
    }
    if ((now_ms - g_last_tx_ms) < HT_CTRL_PERIOD_MS)
    {
        return;
    }
    g_last_tx_ms = now_ms;
    {
        float pos_step = HT_POS_SLEW_RAD_PER_TICK;
        float vel_step = HT_VEL_SLEW_RAD_PER_TICK;
        float tff_step = HT_TFF_SLEW_PER_TICK;

        /* Soft start: reduce slew at enable / large target changes to avoid over-current reset. */
        if ((g_soft_start_until_ms != 0U) && ((int32_t)(now_ms - g_soft_start_until_ms) < 0))
        {
            vel_step = 0.03f;
            tff_step = 0.02f;
        }

        g_cmd_pos = HT_SlewStep(g_cmd_pos, g_target_pos, pos_step);
        g_cmd_vel = HT_SlewStep(g_cmd_vel, g_target_vel, vel_step);
        g_cmd_tff = HT_SlewStep(g_cmd_tff, g_target_tff, tff_step);
    }
    ht_send_cmd(g_cmd_pos, g_cmd_vel, g_kp, g_kd, g_cmd_tff);
}

void HTMotor_UIStart(UI_MotorId_t motor, int16_t target_speed)
{
    float out_rpm = 0.0f;
    float v_motor = 0.0f;
    if (motor != UI_MOTOR_HT)
    {
        return;
    }
    ht_send_special(0xFB); /* clear fault */
    ht_send_special(0xFC); /* enable */
    ht_send_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    ht_send_special(0xFE); /* zero position */
    g_enabled = 1U;
    g_kp = HT_SPEED_KP_DEFAULT;
    g_kd = HT_SPEED_KD_DEFAULT;
    out_rpm = (float)target_speed;
    v_motor = HT_OutputRpmToMotorRadPerSec(out_rpm);
    v_motor = clampf(v_motor, k_v_min, k_v_max);
    g_target_tff = (v_motor > 0.1f) ? HT_SPEED_TFF_DEFAULT : ((v_motor < -0.1f) ? -HT_SPEED_TFF_DEFAULT : 0.0f);
    g_target_pos = 0.0f;
    g_target_vel = v_motor;
    g_cmd_vel = 0.0f;
    g_cmd_tff = 0.0f;
    g_soft_start_until_ms = HAL_GetTick() + 800U;
}

void HTMotor_UISetSpeed(UI_MotorId_t motor, int16_t target_speed)
{
    float out_rpm = 0.0f;
    float v_motor = 0.0f;
    if (motor != UI_MOTOR_HT)
    {
        return;
    }
    g_kp = HT_SPEED_KP_DEFAULT;
    g_kd = HT_SPEED_KD_DEFAULT;
    out_rpm = (float)target_speed;
    v_motor = HT_OutputRpmToMotorRadPerSec(out_rpm);
    v_motor = clampf(v_motor, k_v_min, k_v_max);
    g_target_tff = (v_motor > 0.1f) ? HT_SPEED_TFF_DEFAULT : ((v_motor < -0.1f) ? -HT_SPEED_TFF_DEFAULT : 0.0f);
    g_target_pos = 0.0f;
    if ((v_motor - g_target_vel) > 5.0f || (v_motor - g_target_vel) < -5.0f)
    {
        g_soft_start_until_ms = HAL_GetTick() + 600U;
    }
    g_target_vel = v_motor;
}

void HTMotor_UIStartPos(UI_MotorId_t motor, int16_t target_pos_deg)
{
    if (motor != UI_MOTOR_HT)
    {
        return;
    }
    ht_send_special(0xFB); /* clear fault */
    ht_send_special(0xFC);
    ht_send_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    ht_send_special(0xFE);
    g_enabled = 1U;
    g_kp = HT_POS_KP_DEFAULT;
    g_kd = HT_POS_KD_DEFAULT;
    g_target_tff = HT_POS_TFF_DEFAULT;
    g_target_vel = 0.0f;
    if (g_fb.last_rx_ms != 0U)
    {
        g_cmd_pos = HT_RawPosToMotorRad((uint16_t)g_fb.pos_raw);
    }
    else
    {
        g_cmd_pos = 0.0f;
    }
    g_cmd_vel = 0.0f;
    g_cmd_tff = 0.0f;
    g_target_pos = HT_OutputDegToMotorRad((float)target_pos_deg);
    g_soft_start_until_ms = HAL_GetTick() + 800U;
}

void HTMotor_UISetPos(UI_MotorId_t motor, int16_t target_pos_deg)
{
    if (motor != UI_MOTOR_HT)
    {
        return;
    }
    g_kp = HT_POS_KP_DEFAULT;
    g_kd = HT_POS_KD_DEFAULT;
    g_target_tff = HT_POS_TFF_DEFAULT;
    g_target_vel = 0.0f;
    g_target_pos = HT_OutputDegToMotorRad((float)target_pos_deg);
    g_soft_start_until_ms = HAL_GetTick() + 600U;
}

void HTMotor_UIStop(UI_MotorId_t motor)
{
    if (motor != UI_MOTOR_HT)
    {
        return;
    }
    ht_send_special(0xFD); /* disable */
    g_enabled = 0U;
    g_target_pos = 0.0f;
    g_target_vel = 0.0f;
    g_target_tff = 0.0f;
    g_cmd_pos = 0.0f;
    g_cmd_vel = 0.0f;
    g_cmd_tff = 0.0f;
}

void HTMotor_OnCanRx(uint32_t now_ms, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    uint16_t p = 0U;
    uint16_t v = 0U;
    uint16_t t = 0U;

    if (dlc < 5U)
    {
        return;
    }

    if (data[0] == (uint8_t)HT_MOTOR_CAN_ID)
    {
        /* Vendor demo format: id in data[0] */
        p = (uint16_t)((data[1] << 8) | data[2]);
        v = (uint16_t)((data[3] << 4) | (data[4] >> 4));
        if (dlc >= 6U)
        {
            t = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
        }
    }
    else if (std_id == (uint16_t)HT_MOTOR_CAN_ID)
    {
        /* Standard MIT format: feedback packed from byte0 */
        p = (uint16_t)((data[0] << 8) | data[1]);
        v = (uint16_t)((data[2] << 4) | (data[3] >> 4));
        t = (uint16_t)(((data[3] & 0x0F) << 8) | data[4]);
    }
    else
    {
        return;
    }

    g_fb.pos_raw = (int16_t)p;
    g_fb.vel_raw = (int16_t)v;
    g_fb.tor_raw = (int16_t)t;
    g_fb.last_rx_ms = now_ms;
}

HTMotor_Feedback_t HTMotor_GetFeedback(void)
{
    return g_fb;
}

