#include "ui.h"
#include "ui_test.h"
#include "oled.h"
#include "bsp_can.h"
#include "bad_apple.h"
#include "buzzer.h"
#include "HTMotor.h"
#include "stdio.h"

#define UI_VISIBLE_LINES              5U

extern moto_info_t motor_info[MOTOR_MAX_NUM];

/* Output shaft RPM display ratios (motor:output). Unknown motors default to 1.0. */
#define UI_GEAR_RATIO_GM6020   1.0f
#define UI_GEAR_RATIO_RM3508   1.0f
#define UI_GEAR_RATIO_DM4310   1.0f
#define UI_GEAR_RATIO_ZDT      1.0f

static float UI_GetMotorGearRatio(UI_MotorId_t motor)
{
    switch (motor)
    {
    case UI_MOTOR_GM6020: return UI_GEAR_RATIO_GM6020;
    case UI_MOTOR_RM3508: return UI_GEAR_RATIO_RM3508;
    case UI_MOTOR_DM4310: return UI_GEAR_RATIO_DM4310;
    case UI_MOTOR_ZDT_STEPMOTOR: return UI_GEAR_RATIO_ZDT;
    default: return 1.0f;
    }
}

static uint8_t UI_MotorIdToFeedbackIndex(UI_MotorId_t motor)
{
    switch (motor)
    {
    case UI_MOTOR_GM6020: return 0U;
    case UI_MOTOR_HT: return 0U;
    case UI_MOTOR_RM3508: return 1U;
    case UI_MOTOR_DM4310: return 2U;
    case UI_MOTOR_ZDT_STEPMOTOR: return 3U;
    default: return 0U;
    }
}

static float UI_HTRawSpeedToRad(float raw12)
{
    /* MIT velocity: 12-bit mapped to [-45, 45] rad/s */
    return (raw12 * 90.0f / 4095.0f) - 45.0f;
}

static float UI_RadPerSecToRpm(float rad_s)
{
    return rad_s * 9.5492966f; /* 60 / (2*pi) */
}

static float UI_HTRawPosToDeg(float raw16)
{
    /* HT-03 position: 16-bit mapped to [-95.5, 95.5] rad */
    float pos_rad = (raw16 * 191.0f / 65535.0f) - 95.5f;
    return pos_rad * 57.2957795f;
}

static float UI_GMRawPosToDeg(float raw16)
{
    return (raw16 * 360.0f) / 8192.0f;
}

static void UI_RenderMenu(void)
{
    uint8_t row = 0U;
    char line_buf[22];

    oled_clear(Pen_Clear);
    for (row = 0U; row < UI_VISIBLE_LINES; row++)
    {
        UI_TestGetMenuLine(row, line_buf, sizeof(line_buf));
        oled_showstring(row, 0, (uint8_t *)line_buf);
    }
    oled_refresh_gram();
}

static void UI_RenderSpeedPage(void)
{
    char line[24];
    int16_t speed = UI_TestGetSpeedTarget();
    uint8_t running = UI_TestGetSpeedRunning();
    UI_MotorId_t motor = UI_TestGetSpeedMotorId();
    int16_t speed_fb = 0;
    uint16_t angle_fb = 0;
    uint32_t rx_age = 0xFFFFFFFFU;
    float speed_out_rpm = 0.0f;
    float pos_std = 0.0f;

    if (motor == UI_MOTOR_HT)
    {
        HTMotor_Feedback_t fb = HTMotor_GetFeedback();
        speed_fb = fb.vel_raw;
        angle_fb = (uint16_t)fb.pos_raw;
        /* Convert MIT motor-side rad/s -> output shaft rpm (gear in HTMotor.h). */
        speed_out_rpm = UI_RadPerSecToRpm(UI_HTRawSpeedToRad((float)speed_fb)) / HT_MOTOR_GEAR_RATIO;
        pos_std = UI_HTRawPosToDeg((float)angle_fb);
        if (fb.last_rx_ms > 0U)
        {
            rx_age = HAL_GetTick() - fb.last_rx_ms;
        }
    }
    else
    {
        uint8_t idx = UI_MotorIdToFeedbackIndex(motor);
        speed_fb = motor_info[idx].rotor_speed;
        angle_fb = motor_info[idx].rotor_angle;
    }

    oled_clear(Pen_Clear);

    snprintf(line, sizeof(line), "%s speed", UI_TestGetSpeedMotorName());
    oled_showstring(0, 0, (uint8_t *)line);

    snprintf(line, sizeof(line), "tar:%d rpm %s", speed, running ? "ON" : "OFF");
    oled_showstring(1, 0, (uint8_t *)line);

    if (motor == UI_MOTOR_HT)
    {
        snprintf(line, sizeof(line), "spd_fb:%0.0f rpm", speed_out_rpm);
        oled_showstring(2, 0, (uint8_t *)line);

        snprintf(line, sizeof(line), "pos:%0.1f deg", pos_std);
        oled_showstring(3, 0, (uint8_t *)line);

        if (rx_age == 0xFFFFFFFFU)
        {
            snprintf(line, sizeof(line), "C:set LC:no_rx");
        }
        else
        {
            snprintf(line, sizeof(line), "C:set LC:%lums", (unsigned long)rx_age);
        }
        oled_showstring(4, 0, (uint8_t *)line);
    }
    else
    {
        float ratio = UI_GetMotorGearRatio(motor);
        speed_out_rpm = (ratio > 0.0f) ? ((float)speed_fb / ratio) : (float)speed_fb;
        snprintf(line, sizeof(line), "out:%0.0f rpm", speed_out_rpm);
        oled_showstring(2, 0, (uint8_t *)line);

        snprintf(line, sizeof(line), "enc(raw):%u", (unsigned int)angle_fb);
        oled_showstring(3, 0, (uint8_t *)line);
        oled_showstring(4, 0, (uint8_t *)"L/R:spd C:set LC:bk");
    }

    oled_refresh_gram();
}

static void UI_RenderPosPage(void)
{
    char line[24];
    UI_MotorId_t motor = UI_TestGetPosMotorId();
    int16_t pos_deg = UI_TestGetPosTargetDeg();
    uint8_t running = UI_TestGetPosRunning();
    int16_t speed_fb = 0;
    uint16_t angle_fb = 0;
    float speed_out_rpm = 0.0f;
    float pos_std = 0.0f;
    uint32_t rx_age = 0xFFFFFFFFU;

    oled_clear(Pen_Clear);

    snprintf(line, sizeof(line), "%s pos", UI_TestGetPosMotorName());
    oled_showstring(0, 0, (uint8_t *)line);

    snprintf(line, sizeof(line), "tar:%d %s", pos_deg, running ? "ON" : "OFF");
    oled_showstring(1, 0, (uint8_t *)line);

    if (motor == UI_MOTOR_HT)
    {
        HTMotor_Feedback_t fb = HTMotor_GetFeedback();
        speed_fb = fb.vel_raw;
        angle_fb = (uint16_t)fb.pos_raw;
        speed_out_rpm = UI_RadPerSecToRpm(UI_HTRawSpeedToRad((float)speed_fb)) / HT_MOTOR_GEAR_RATIO;
        pos_std = UI_HTRawPosToDeg((float)angle_fb);
        if (fb.last_rx_ms > 0U)
        {
            rx_age = HAL_GetTick() - fb.last_rx_ms;
        }

        snprintf(line, sizeof(line), "spd_fb:%0.0f rpm", speed_out_rpm);
        oled_showstring(2, 0, (uint8_t *)line);
        snprintf(line, sizeof(line), "pos:%0.1f deg", pos_std);
        oled_showstring(3, 0, (uint8_t *)line);
        if (rx_age == 0xFFFFFFFFU)
        {
            oled_showstring(4, 0, (uint8_t *)"L/R:30 C:set no_rx");
        }
        else
        {
            snprintf(line, sizeof(line), "L/R:30 C:%lums", (unsigned long)rx_age);
            oled_showstring(4, 0, (uint8_t *)line);
        }
    }
    else
    {
        uint8_t idx = UI_MotorIdToFeedbackIndex(motor);
        speed_fb = motor_info[idx].rotor_speed;
        angle_fb = motor_info[idx].rotor_angle;
        {
            float ratio = UI_GetMotorGearRatio(motor);
            speed_out_rpm = (ratio > 0.0f) ? ((float)speed_fb / ratio) : (float)speed_fb;
        }
        pos_std = UI_GMRawPosToDeg((float)angle_fb);

        snprintf(line, sizeof(line), "spd_fb:%0.0f rpm", speed_out_rpm);
        oled_showstring(2, 0, (uint8_t *)line);
        snprintf(line, sizeof(line), "pos:%0.1f deg", pos_std);
        oled_showstring(3, 0, (uint8_t *)line);
        oled_showstring(4, 0, (uint8_t *)"L/R:30 C:set LC:bk");
    }

    oled_refresh_gram();
}

static void UI_RenderPlaceholder(const char *title)
{
    oled_clear(Pen_Clear);
    oled_showstring(0, 0, (uint8_t *)title);
    oled_showstring(2, 0, (uint8_t *)"TODO");
    oled_showstring(4, 0, (uint8_t *)"LongC back");
    oled_refresh_gram();
}

static void UI_Render(void)
{
    UI_TestView_t view = UI_TestGetView();
    if (view == UI_TEST_VIEW_SPEED)
    {
        UI_RenderSpeedPage();
        return;
    }
    if (view == UI_TEST_VIEW_POS)
    {
        UI_RenderPosPage();
        return;
    }
    if (view == UI_TEST_VIEW_OLED_TEST)
    {
        BadApple_DrawFrame(HAL_GetTick());
        return;
    }

    if (view == UI_TEST_VIEW_PLACEHOLDER)
    {
        UI_RenderPlaceholder(UI_TestGetPlaceholderTitle());
        return;
    }

    UI_RenderMenu();
}

void UI_Init(const UI_Callbacks_t *callbacks)
{
    UI_TestInit(callbacks);
    BadApple_Reset();
}

void UI_Task(uint32_t now_ms, ButtonAD_Key_t key_now)
{
    UI_TestTask(now_ms, key_now);
    UI_TestView_t view = UI_TestGetView();
    Buzzer_Task(now_ms, (view == UI_TEST_VIEW_OLED_TEST) ? 1U : 0U);
    if (UI_TestConsumeRedraw())
    {
        UI_Render();
    }
}

void UI_RequestRedraw(void)
{
    UI_TestRequestRedraw();
}
