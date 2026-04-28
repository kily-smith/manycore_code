#include "ui_test.h"

#include <stdio.h>
#include <string.h>

#define UI_VISIBLE_LINES              5U
#define UI_CENTER_LONG_PRESS_MS       800U
#define UI_SPEED_MIN                  (-5000)
#define UI_SPEED_MAX                  (5000)
#define UI_SPEED_STEP                 (50)
#define UI_POS_MIN_DEG                (-180)
#define UI_POS_MAX_DEG                (180)
#define UI_POS_STEP_DEG               (30)
#define UI_LIVE_REFRESH_MS            (50U)

typedef enum
{
    UI_NODE_MENU = 0,
    UI_NODE_SPEED_PAGE,
    UI_NODE_POS_PAGE,
    UI_NODE_OLED_TEST,
    UI_NODE_USART_TEST,
    UI_NODE_BUZZER_TEST,
    UI_NODE_LED_TEST
} UI_NodeType_t;

typedef struct UI_MenuNode UI_MenuNode_t;
struct UI_MenuNode
{
    const char *title;
    UI_NodeType_t type;
    UI_MotorId_t motor;
    const UI_MenuNode_t *children;
    uint8_t child_count;
};

typedef enum
{
    UI_EVT_NONE = 0,
    UI_EVT_UP,
    UI_EVT_DOWN,
    UI_EVT_LEFT,
    UI_EVT_RIGHT,
    UI_EVT_CENTER_SHORT,
    UI_EVT_CENTER_LONG
} UI_Event_t;

static const UI_MenuNode_t g_motor_param_nodes[] =
{
    {"speed", UI_NODE_SPEED_PAGE, UI_MOTOR_GM6020, 0, 0},
    {"pos",   UI_NODE_POS_PAGE,   UI_MOTOR_GM6020, 0, 0},
};

static const UI_MenuNode_t g_motor_nodes[] =
{
    {"GM6020",        UI_NODE_MENU, UI_MOTOR_GM6020, g_motor_param_nodes, 2},
    {"HT motor",      UI_NODE_MENU, UI_MOTOR_HT,     g_motor_param_nodes, 2},
    {"RM3508",        UI_NODE_MENU, UI_MOTOR_RM3508, g_motor_param_nodes, 2},
    {"DM4310",        UI_NODE_MENU, UI_MOTOR_DM4310, g_motor_param_nodes, 2},
    {"ZDT StepMotor", UI_NODE_MENU, UI_MOTOR_ZDT_STEPMOTOR, g_motor_param_nodes, 2},
};

static const UI_MenuNode_t g_root_nodes[] =
{
    {"motor test",  UI_NODE_MENU,        UI_MOTOR_GM6020, g_motor_nodes, 5},
    {"oled_test",   UI_NODE_OLED_TEST,   UI_MOTOR_GM6020, 0, 0},
    {"usart test",  UI_NODE_USART_TEST,  UI_MOTOR_GM6020, 0, 0},
    {"buzzer test", UI_NODE_BUZZER_TEST, UI_MOTOR_GM6020, 0, 0},
    {"led test",    UI_NODE_LED_TEST,    UI_MOTOR_GM6020, 0, 0},
};

static const UI_MenuNode_t g_root_menu =
{
    "root",
    UI_NODE_MENU,
    UI_MOTOR_GM6020,
    g_root_nodes,
    (uint8_t)(sizeof(g_root_nodes) / sizeof(g_root_nodes[0]))
};

static UI_Callbacks_t g_cb = {0};
static const UI_MenuNode_t *g_menu_stack[4] = {0};
static uint8_t g_select_stack[4] = {0};
static uint8_t g_depth = 0;
static uint8_t g_need_redraw = 1U;

static uint8_t g_center_pressed = 0U;
static uint32_t g_center_press_start_ms = 0U;
static uint8_t g_center_long_sent = 0U;
static ButtonAD_Key_t g_last_key = BUTTON_AD_NONE;

static uint8_t g_in_speed_page = 0U;
static uint8_t g_in_pos_page = 0U;
static uint8_t g_in_oled_test_page = 0U;
static const UI_MenuNode_t *g_page_node = 0;
static UI_MotorId_t g_speed_motor = UI_MOTOR_GM6020;
static int16_t g_speed_target = 0;
static uint8_t g_speed_running = 0U;
static UI_MotorId_t g_pos_motor = UI_MOTOR_GM6020;
static int16_t g_pos_target_deg = 0;
static uint8_t g_pos_running = 0U;
static uint32_t g_live_refresh_last_ms = 0U;

static const char *UI_TestGetMotorName(UI_MotorId_t motor)
{
    switch (motor)
    {
    case UI_MOTOR_GM6020: return "GM6020";
    case UI_MOTOR_HT: return "HT motor";
    case UI_MOTOR_RM3508: return "RM3508";
    case UI_MOTOR_DM4310: return "DM4310";
    case UI_MOTOR_ZDT_STEPMOTOR: return "ZDT StepMotor";
    default: return "unknown";
    }
}

static void UI_TestClampSpeed(void)
{
    if (g_speed_target < UI_SPEED_MIN) g_speed_target = UI_SPEED_MIN;
    if (g_speed_target > UI_SPEED_MAX) g_speed_target = UI_SPEED_MAX;
}

static void UI_TestClampPos(void)
{
    if (g_pos_target_deg < UI_POS_MIN_DEG) g_pos_target_deg = UI_POS_MIN_DEG;
    if (g_pos_target_deg > UI_POS_MAX_DEG) g_pos_target_deg = UI_POS_MAX_DEG;
}

static void UI_TestEnterNode(const UI_MenuNode_t *node)
{
    /* Enter a menu node or open a function page. */
    if (node->type == UI_NODE_MENU)
    {
        if (g_depth < 3U)
        {
            g_depth++;
            g_menu_stack[g_depth] = node;
            g_select_stack[g_depth] = 0U;
        }
        g_need_redraw = 1U;
        return;
    }

    if (node->type == UI_NODE_SPEED_PAGE)
    {
        g_in_speed_page = 1U;
        g_in_pos_page = 0U;
        g_in_oled_test_page = 0U;
        g_speed_motor = g_menu_stack[g_depth]->motor;
        g_speed_target = 0;
        g_speed_running = 0U;
        g_need_redraw = 1U;
        return;
    }

    if (node->type == UI_NODE_POS_PAGE)
    {
        g_in_pos_page = 1U;
        g_in_speed_page = 0U;
        g_in_oled_test_page = 0U;
        g_pos_motor = g_menu_stack[g_depth]->motor;
        g_pos_target_deg = 0;
        g_pos_running = 0U;
        g_need_redraw = 1U;
        return;
    }

    if (node->type == UI_NODE_OLED_TEST)
    {
        g_in_oled_test_page = 1U;
        g_in_speed_page = 0U;
        g_in_pos_page = 0U;
        g_need_redraw = 1U;
        return;
    }

    g_page_node = node;
    g_need_redraw = 1U;
}

static void UI_TestLeaveCurrent(void)
{
    /* Leave current page. If motor is running, stop it first. */
    if (g_in_speed_page)
    {
        if (g_speed_running && (g_cb.on_motor_stop != 0))
        {
            g_cb.on_motor_stop(g_speed_motor);
        }
        g_in_speed_page = 0U;
        g_speed_running = 0U;
        g_need_redraw = 1U;
        return;
    }

    if (g_in_oled_test_page)
    {
        g_in_oled_test_page = 0U;
        g_need_redraw = 1U;
        return;
    }

    if (g_in_pos_page)
    {
        if (g_pos_running && (g_cb.on_motor_stop != 0))
        {
            g_cb.on_motor_stop(g_pos_motor);
        }
        g_in_pos_page = 0U;
        g_pos_running = 0U;
        g_need_redraw = 1U;
        return;
    }
    if (g_page_node != 0)
    {
        g_page_node = 0;
        g_need_redraw = 1U;
        return;
    }

    if (g_depth > 0U)
    {
        g_depth--;
        g_need_redraw = 1U;
    }
}

static UI_Event_t UI_TestPollEvent(uint32_t now_ms, ButtonAD_Key_t key_now)
{
    UI_Event_t evt = UI_EVT_NONE;

    /* Center key: short press / long press detection. */
    if (key_now == BUTTON_AD_CENTER)
    {
        if (!g_center_pressed)
        {
            g_center_pressed = 1U;
            g_center_press_start_ms = now_ms;
            g_center_long_sent = 0U;
        }
        else if ((!g_center_long_sent) && ((now_ms - g_center_press_start_ms) >= UI_CENTER_LONG_PRESS_MS))
        {
            g_center_long_sent = 1U;
            evt = UI_EVT_CENTER_LONG;
        }
    }
    else if (g_center_pressed)
    {
        if (!g_center_long_sent)
        {
            evt = UI_EVT_CENTER_SHORT;
        }
        g_center_pressed = 0U;
        g_center_long_sent = 0U;
    }

    /* Direction keys are edge-triggered (only on key change). */
    if ((evt == UI_EVT_NONE) && (key_now != g_last_key))
    {
        if (key_now == BUTTON_AD_UP) evt = UI_EVT_UP;
        else if (key_now == BUTTON_AD_DOWN) evt = UI_EVT_DOWN;
        else if (key_now == BUTTON_AD_LEFT) evt = UI_EVT_LEFT;
        else if (key_now == BUTTON_AD_RIGHT) evt = UI_EVT_RIGHT;
    }

    g_last_key = key_now;
    return evt;
}

void UI_TestInit(const UI_Callbacks_t *callbacks)
{
    memset(&g_cb, 0, sizeof(g_cb));
    if (callbacks != 0)
    {
        g_cb = *callbacks;
    }

    g_menu_stack[0] = &g_root_menu;
    g_select_stack[0] = 0U;
    g_depth = 0U;
    g_in_speed_page = 0U;
    g_in_pos_page = 0U;
    g_in_oled_test_page = 0U;
    g_page_node = 0;
    g_speed_running = 0U;
    g_speed_target = 0;
    g_pos_running = 0U;
    g_pos_target_deg = 0;
    g_need_redraw = 1U;
    g_center_pressed = 0U;
    g_center_long_sent = 0U;
    g_last_key = BUTTON_AD_NONE;
    g_live_refresh_last_ms = 0U;
}

void UI_TestTask(uint32_t now_ms, ButtonAD_Key_t key_now)
{
    UI_Event_t evt = UI_TestPollEvent(now_ms, key_now);
    const UI_MenuNode_t *menu = g_menu_stack[g_depth];
    uint8_t *selected = &g_select_stack[g_depth];

    if (evt == UI_EVT_NONE)
    {
        /* Live refresh in speed/pos pages, even without key input. */
        if ((g_in_speed_page || g_in_pos_page || g_in_oled_test_page) && ((now_ms - g_live_refresh_last_ms) >= UI_LIVE_REFRESH_MS))
        {
            g_live_refresh_last_ms = now_ms;
            g_need_redraw = 1U;
        }
        return;
    }

    /* Speed page interaction: adjust target and apply by center short. */
    if (g_in_speed_page)
    {
        if ((evt == UI_EVT_LEFT) || (evt == UI_EVT_DOWN))
        {
            g_speed_target -= UI_SPEED_STEP;
            UI_TestClampSpeed();
            g_need_redraw = 1U;
        }
        else if ((evt == UI_EVT_RIGHT) || (evt == UI_EVT_UP))
        {
            g_speed_target += UI_SPEED_STEP;
            UI_TestClampSpeed();
            g_need_redraw = 1U;
        }
        else if (evt == UI_EVT_CENTER_SHORT)
        {
            if (!g_speed_running)
            {
                g_speed_running = 1U;
                if (g_cb.on_motor_start != 0)
                {
                    g_cb.on_motor_start(g_speed_motor, g_speed_target);
                }
            }
            else if (g_cb.on_motor_set_speed != 0)
            {
                g_cb.on_motor_set_speed(g_speed_motor, g_speed_target);
            }
            g_need_redraw = 1U;
        }
        else if (evt == UI_EVT_CENTER_LONG)
        {
            UI_TestLeaveCurrent();
        }
    }
    /* Position page interaction: adjust target degree and apply by center short. */
    else if (g_in_pos_page)
    {
        if ((evt == UI_EVT_LEFT) || (evt == UI_EVT_DOWN))
        {
            g_pos_target_deg -= UI_POS_STEP_DEG;
            UI_TestClampPos();
            g_need_redraw = 1U;
        }
        else if ((evt == UI_EVT_RIGHT) || (evt == UI_EVT_UP))
        {
            g_pos_target_deg += UI_POS_STEP_DEG;
            UI_TestClampPos();
            g_need_redraw = 1U;
        }
        else if (evt == UI_EVT_CENTER_SHORT)
        {
            if (!g_pos_running)
            {
                g_pos_running = 1U;
                if (g_cb.on_motor_start_pos != 0)
                {
                    g_cb.on_motor_start_pos(g_pos_motor, g_pos_target_deg);
                }
            }
            else if (g_cb.on_motor_set_pos != 0)
            {
                g_cb.on_motor_set_pos(g_pos_motor, g_pos_target_deg);
            }
            g_need_redraw = 1U;
        }
        else if (evt == UI_EVT_CENTER_LONG)
        {
            UI_TestLeaveCurrent();
        }
    }
    else if (g_in_oled_test_page)
    {
        if (evt == UI_EVT_CENTER_LONG)
        {
            UI_TestLeaveCurrent();
        }
    }
    else
    {
        /* Menu interaction: move selection and enter/leave pages. */
        if ((evt == UI_EVT_UP) || (evt == UI_EVT_LEFT))
        {
            if (*selected > 0U)
            {
                (*selected)--;
                g_need_redraw = 1U;
            }
        }
        else if ((evt == UI_EVT_DOWN) || (evt == UI_EVT_RIGHT))
        {
            if ((*selected + 1U) < menu->child_count)
            {
                (*selected)++;
                g_need_redraw = 1U;
            }
        }
        else if (evt == UI_EVT_CENTER_SHORT)
        {
            UI_TestEnterNode(&menu->children[*selected]);
        }
        else if (evt == UI_EVT_CENTER_LONG)
        {
            UI_TestLeaveCurrent();
        }
    }
}

void UI_TestRequestRedraw(void)
{
    g_need_redraw = 1U;
}

uint8_t UI_TestConsumeRedraw(void)
{
    uint8_t need = g_need_redraw;
    g_need_redraw = 0U;
    return need;
}

UI_TestView_t UI_TestGetView(void)
{
    if (g_in_speed_page)
    {
        return UI_TEST_VIEW_SPEED;
    }
    if (g_in_pos_page)
    {
        return UI_TEST_VIEW_POS;
    }
    if (g_in_oled_test_page)
    {
        return UI_TEST_VIEW_OLED_TEST;
    }
    if (g_page_node != 0)
    {
        return UI_TEST_VIEW_PLACEHOLDER;
    }
    return UI_TEST_VIEW_MENU;
}

void UI_TestGetMenuLine(uint8_t row, char *buf, uint32_t buf_len)
{
    const UI_MenuNode_t *menu = g_menu_stack[g_depth];
    uint8_t selected = g_select_stack[g_depth];
    uint8_t top = 0U;
    uint8_t index = 0U;

    if ((buf == 0) || (buf_len == 0U))
    {
        return;
    }

    buf[0] = '\0';

    if (selected >= UI_VISIBLE_LINES)
    {
        top = (uint8_t)(selected - (UI_VISIBLE_LINES - 1U));
    }

    index = (uint8_t)(top + row);
    if (index >= menu->child_count)
    {
        return;
    }

    snprintf(buf, buf_len, "%c%s", (index == selected) ? '>' : ' ', menu->children[index].title);
}

const char *UI_TestGetPlaceholderTitle(void)
{
    if (g_page_node == 0)
    {
        return "";
    }
    return g_page_node->title;
}

const char *UI_TestGetSpeedMotorName(void)
{
    return UI_TestGetMotorName(g_speed_motor);
}

UI_MotorId_t UI_TestGetSpeedMotorId(void)
{
    return g_speed_motor;
}

int16_t UI_TestGetSpeedTarget(void)
{
    return g_speed_target;
}

uint8_t UI_TestGetSpeedRunning(void)
{
    return g_speed_running;
}

const char *UI_TestGetPosMotorName(void)
{
    return UI_TestGetMotorName(g_pos_motor);
}

UI_MotorId_t UI_TestGetPosMotorId(void)
{
    return g_pos_motor;
}

int16_t UI_TestGetPosTargetDeg(void)
{
    return g_pos_target_deg;
}

uint8_t UI_TestGetPosRunning(void)
{
    return g_pos_running;
}
