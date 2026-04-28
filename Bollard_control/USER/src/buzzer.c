#include "buzzer.h"

#include "tim.h"

#define BUZZER_VOLUME_DIV 4U
#define BUZZER_MIN_FREQ_HZ 220U
#define BUZZER_MAX_FREQ_HZ 3000U
#define BUZZER_MIN_NOTE_MS 35U

typedef struct
{
    uint16_t freq_hz;
    uint16_t dur_ms;
} BuzzerNote_t;

/* Bad Apple!! (intro motif, simplified monophonic) */
static const BuzzerNote_t g_tune[] =
{
    /* Slow intro arpeggio-like motif */
    {392, 300}, {494, 300}, {587, 300}, {784, 450}, {0, 120},
    {392, 300}, {494, 300}, {587, 300}, {740, 450}, {0, 120},
    {392, 300}, {466, 300}, {587, 300}, {698, 450}, {0, 180},

    /* Repeat with a small variation */
    {349, 300}, {440, 300}, {523, 300}, {659, 450}, {0, 120},
    {349, 300}, {440, 300}, {523, 300}, {622, 450}, {0, 120},
    {349, 300}, {415, 300}, {523, 300}, {587, 520}, {0, 220},
};

static uint8_t g_pwm_started = 0U;
static uint8_t g_playing = 0U;
static uint16_t g_note_idx = 0U;
static uint32_t g_note_deadline_ms = 0U;
static uint32_t g_tim_cnt_hz = 0U;

static uint16_t Buzzer_FilterFreq(uint16_t freq_hz)
{
    if ((freq_hz < BUZZER_MIN_FREQ_HZ) || (freq_hz > BUZZER_MAX_FREQ_HZ))
    {
        return 0U;
    }
    return freq_hz;
}

static uint32_t Buzzer_GetTim12ClkHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

    /* If APB prescaler != 1, timer clock = 2 * PCLK. */
    if (ppre1 >= 4U)
    {
        return pclk1 * 2U;
    }
    return pclk1;
}

static void Buzzer_ApplyPwm(uint16_t arr, uint16_t ccr)
{
    /* Mute first to reduce clicks, then update ARR/CCR atomically with UG. */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
    __HAL_TIM_SET_COUNTER(&htim12, 0U);
    htim12.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, ccr);
}

static void Buzzer_SetFreq(uint16_t freq_hz)
{
    uint32_t arr = 0U;
    uint32_t ccr = 0U;

    if (freq_hz == 0U)
    {
        Buzzer_ApplyPwm(__HAL_TIM_GET_AUTORELOAD(&htim12), 0U);
        return;
    }

    if (g_tim_cnt_hz == 0U)
    {
        return;
    }

    arr = (g_tim_cnt_hz / freq_hz);
    if (arr > 0U)
    {
        arr -= 1U;
    }
    if (arr > 65535U)
    {
        arr = 65535U;
    }

    ccr = (arr + 1U) / BUZZER_VOLUME_DIV;
    Buzzer_ApplyPwm((uint16_t)arr, (uint16_t)ccr);
}

void Buzzer_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    g_tim_cnt_hz = Buzzer_GetTim12ClkHz() / ((uint32_t)htim12.Instance->PSC + 1U);
    Buzzer_ApplyPwm(__HAL_TIM_GET_AUTORELOAD(&htim12), 0U);
    g_pwm_started = 1U;
    g_playing = 0U;
    g_note_idx = 0U;
    g_note_deadline_ms = 0U;
}

void Buzzer_Task(uint32_t now_ms, uint8_t enable_play)
{
    if (!g_pwm_started)
    {
        return;
    }

    if (!enable_play)
    {
        if (g_playing)
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0U);
            g_playing = 0U;
            g_note_idx = 0U;
            g_note_deadline_ms = 0U;
        }
        return;
    }

    if (!g_playing)
    {
        g_playing = 1U;
        g_note_idx = 0U;
        g_note_deadline_ms = now_ms;
    }

    if ((int32_t)(now_ms - g_note_deadline_ms) >= 0)
    {
        const BuzzerNote_t *n = &g_tune[g_note_idx];
        uint16_t filtered_freq = Buzzer_FilterFreq(n->freq_hz);
        uint16_t dur_ms = n->dur_ms;
        if (dur_ms < BUZZER_MIN_NOTE_MS)
        {
            dur_ms = BUZZER_MIN_NOTE_MS;
        }

        Buzzer_SetFreq(filtered_freq);
        g_note_deadline_ms = now_ms + dur_ms;
        g_note_idx++;
        if (g_note_idx >= (uint16_t)(sizeof(g_tune) / sizeof(g_tune[0])))
        {
            g_note_idx = 0U;
        }
    }
}
