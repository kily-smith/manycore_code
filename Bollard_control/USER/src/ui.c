#include "ui.h"
#include "oled.h"
#include "usart.h"
#include "stdio.h"

#define UI_REFRESH_PERIOD_MS          100U
#define UI_DOUBLE_CLICK_MS            350U
#define UI_HEIGHT_STEP_MM             10
#define UI_HEIGHT_MAX_MM              400
#define UI_HEIGHT_MIN_MM              0

static uint8_t g_ui_need_redraw = 1U;
static uint32_t g_ui_last_refresh_ms = 0U;
static ButtonAD_Key_t g_last_key = BUTTON_AD_NONE;
static uint32_t g_last_center_release_ms = 0U;
static uint8_t g_center_wait_second = 0U;
static uint8_t g_dir_up = 1U;
static int32_t g_height_setting_mm = 0;

static void UI_ApplySetting(void)
{
    EmmMotor_SetHeightSettingMm(g_height_setting_mm, g_dir_up);
}

static void UI_OnKeyEvent(uint32_t now_ms, ButtonAD_Key_t key)
{
    if (key == BUTTON_AD_LEFT)
    {
        g_dir_up = 0U;
        UI_ApplySetting();
        g_ui_need_redraw = 1U;
    }
    else if (key == BUTTON_AD_RIGHT)
    {
        g_dir_up = 1U;
        UI_ApplySetting();
        g_ui_need_redraw = 1U;
    }
    else if (key == BUTTON_AD_UP)
    {
        g_height_setting_mm += UI_HEIGHT_STEP_MM;
        if (g_height_setting_mm > UI_HEIGHT_MAX_MM)
        {
            g_height_setting_mm = UI_HEIGHT_MAX_MM;
        }
        UI_ApplySetting();
        g_ui_need_redraw = 1U;
    }
    else if (key == BUTTON_AD_DOWN)
    {
        g_height_setting_mm -= UI_HEIGHT_STEP_MM;
        if (g_height_setting_mm < UI_HEIGHT_MIN_MM)
        {
            g_height_setting_mm = UI_HEIGHT_MIN_MM;
        }
        UI_ApplySetting();
        g_ui_need_redraw = 1U;
    }
    else if (key == BUTTON_AD_CENTER)
    {
        if ((g_center_wait_second != 0U) && ((now_ms - g_last_center_release_ms) <= UI_DOUBLE_CLICK_MS))
        {
            g_center_wait_second = 0U;
            EmmMotor_StopMove();
        }
        else
        {
            EmmMotor_StartMoveToSetting();
            g_center_wait_second = 1U;
            g_last_center_release_ms = now_ms;
        }
        g_ui_need_redraw = 1U;
    }
}

static void UI_Render(void)
{
    char line[24];
    float pos_deg = EmmMotor_GetCurrentPosDeg();
    float speed_rpm = EmmMotor_GetSpeedRpm();
    float target_pos_deg = EmmMotor_GetTargetPosDeg();
    const char *dir_str = (g_dir_up != 0U) ? "UP" : "DOWN";

    oled_clear(Pen_Clear);
    oled_showstring(0, 0, (uint8_t *)"ZDT_Stepmotor");

    snprintf(line, sizeof(line), "Pos:%0.1f deg", pos_deg);
    oled_showstring(1, 0, (uint8_t *)line);

    snprintf(line, sizeof(line), "ReadPos:%0.1f", pos_deg);
    oled_showstring(2, 0, (uint8_t *)line);

    snprintf(line, sizeof(line), "Speed:%0.1f rpm", speed_rpm);
    oled_showstring(3, 0, (uint8_t *)line);

    (void)target_pos_deg;
    snprintf(line, sizeof(line), "%s H:%ldmm step:%d", dir_str, (long)g_height_setting_mm, UI_HEIGHT_STEP_MM);
    oled_showstring(4, 0, (uint8_t *)line);

    oled_refresh_gram();
}

void UI_Init(const UI_Callbacks_t *callbacks)
{
    (void)callbacks;
    g_ui_need_redraw = 1U;
    g_ui_last_refresh_ms = 0U;
    g_last_key = BUTTON_AD_NONE;
    g_last_center_release_ms = 0U;
    g_center_wait_second = 0U;
    g_dir_up = EmmMotor_GetDirUp();
    g_height_setting_mm = EmmMotor_GetHeightSettingMm();
    UI_ApplySetting();
}

void UI_Task(uint32_t now_ms, ButtonAD_Key_t key_now)
{
    if ((key_now != BUTTON_AD_NONE) && (key_now != g_last_key))
    {
        UI_OnKeyEvent(now_ms, key_now);
    }
    if ((key_now == BUTTON_AD_NONE) && (g_last_key == BUTTON_AD_CENTER))
    {
        g_last_center_release_ms = now_ms;
    }
    if ((g_center_wait_second != 0U) && ((now_ms - g_last_center_release_ms) > UI_DOUBLE_CLICK_MS))
    {
        g_center_wait_second = 0U;
    }
    g_last_key = key_now;

    if ((g_ui_need_redraw != 0U) || ((now_ms - g_ui_last_refresh_ms) >= UI_REFRESH_PERIOD_MS))
    {
        g_ui_need_redraw = 0U;
        g_ui_last_refresh_ms = now_ms;
        UI_Render();
    }
}

void UI_RequestRedraw(void)
{
    g_ui_need_redraw = 1U;
}

void UI_SetTargetPositionDeg(int32_t target_pos_deg)
{
    (void)target_pos_deg;
    g_ui_need_redraw = 1U;
}

int32_t UI_GetTargetPositionDeg(void)
{
    return (int32_t)EmmMotor_GetTargetPosDeg();
}
