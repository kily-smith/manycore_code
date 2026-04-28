#include "Button_AD.h"
#include "adc.h"
#include "main.h"

#define BUTTON_AD_SAMPLE_COUNT           5U
#define BUTTON_AD_DEBOUNCE_MS            25U
#define BUTTON_AD_REARM_RELEASE_MS       40U

/* Based on the provided 12-bit reference values with tolerance */
#define BUTTON_AD_CENTER_MAX  300U
#define BUTTON_AD_LEFT_MIN    650U
#define BUTTON_AD_LEFT_MAX    1050U
#define BUTTON_AD_RIGHT_MIN   1550U
#define BUTTON_AD_RIGHT_MAX   1950U
#define BUTTON_AD_UP_MIN      2250U
#define BUTTON_AD_UP_MAX      2650U
#define BUTTON_AD_DOWN_MIN    3050U
#define BUTTON_AD_DOWN_MAX    3500U

static ButtonAD_Key_t s_raw_last = BUTTON_AD_NONE;
static uint32_t s_raw_change_ms = 0U;

static ButtonAD_Key_t s_stable = BUTTON_AD_NONE;
static uint8_t s_armed = 1U;
static uint32_t s_last_release_ms = 0U;

void ButtonAD_Init(void)
{
    s_raw_last = BUTTON_AD_NONE;
    s_raw_change_ms = HAL_GetTick();
    s_stable = BUTTON_AD_NONE;
    s_armed = 1U;
    s_last_release_ms = HAL_GetTick();
}

uint16_t ButtonAD_ReadRaw(void)
{
    uint16_t sample[BUTTON_AD_SAMPLE_COUNT];
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint16_t tmp = 0U;

    for (i = 0U; i < BUTTON_AD_SAMPLE_COUNT; i++)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK)
        {
            sample[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }
        else
        {
            sample[i] = 0U;
        }
        HAL_ADC_Stop(&hadc1);
    }

    for (i = 0U; i < (BUTTON_AD_SAMPLE_COUNT - 1U); i++)
    {
        for (j = 0U; j < (BUTTON_AD_SAMPLE_COUNT - 1U - i); j++)
        {
            if (sample[j] > sample[j + 1U])
            {
                tmp = sample[j];
                sample[j] = sample[j + 1U];
                sample[j + 1U] = tmp;
            }
        }
    }

    return sample[BUTTON_AD_SAMPLE_COUNT / 2U];
}

ButtonAD_Key_t ButtonAD_ReadKey(void)
{
    uint16_t ad = ButtonAD_ReadRaw();
    ButtonAD_Key_t raw = BUTTON_AD_NONE;
    uint32_t now_ms = HAL_GetTick();

    if (ad <= BUTTON_AD_CENTER_MAX)
    {
        raw = BUTTON_AD_CENTER;
    }
    else if ((ad >= BUTTON_AD_LEFT_MIN) && (ad <= BUTTON_AD_LEFT_MAX))
    {
        raw = BUTTON_AD_LEFT;
    }
    else if ((ad >= BUTTON_AD_RIGHT_MIN) && (ad <= BUTTON_AD_RIGHT_MAX))
    {
        raw = BUTTON_AD_RIGHT;
    }
    else if ((ad >= BUTTON_AD_UP_MIN) && (ad <= BUTTON_AD_UP_MAX))
    {
        raw = BUTTON_AD_UP;
    }
    else if ((ad >= BUTTON_AD_DOWN_MIN) && (ad <= BUTTON_AD_DOWN_MAX))
    {
        raw = BUTTON_AD_DOWN;
    }
    else
    {
        raw = BUTTON_AD_NONE;
    }

    if (raw != s_raw_last)
    {
        s_raw_last = raw;
        s_raw_change_ms = now_ms;
    }

    /* Update stable key after it holds for debounce time. */
    if ((raw != s_stable) && ((now_ms - s_raw_change_ms) >= BUTTON_AD_DEBOUNCE_MS))
    {
        s_stable = raw;
        if (s_stable == BUTTON_AD_NONE)
        {
            s_last_release_ms = now_ms;
        }
    }

    /* Rearm only after a real release for some time (prevents long-press repeats). */
    if ((s_stable == BUTTON_AD_NONE) && ((now_ms - s_last_release_ms) >= BUTTON_AD_REARM_RELEASE_MS))
    {
        s_armed = 1U;
    }

    return s_stable;
}

ButtonAD_Key_t ButtonAD_GetEvent(void)
{
    ButtonAD_Key_t stable = ButtonAD_ReadKey();

    if ((stable != BUTTON_AD_NONE) && (s_armed != 0U))
    {
        s_armed = 0U;
        return stable;
    }
    return BUTTON_AD_NONE;
}
