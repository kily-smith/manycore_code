#include "Button_AD.h"
#include "adc.h"

#define BUTTON_AD_SAMPLE_COUNT      9U
#define BUTTON_AD_DEBOUNCE_COUNT    4U
#define BUTTON_AD_RELEASE_COUNT     3U

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

static ButtonAD_Key_t button_last_raw = BUTTON_AD_NONE;
static ButtonAD_Key_t button_stable = BUTTON_AD_NONE;
static uint8_t button_same_count = 0U;
static uint8_t button_release_count = 0U;
static uint8_t button_armed = 1U;

static ButtonAD_Key_t readkey_last_raw = BUTTON_AD_NONE;
static ButtonAD_Key_t readkey_stable = BUTTON_AD_NONE;
static uint8_t readkey_same_count = 0U;

void ButtonAD_Init(void)
{
    button_last_raw = BUTTON_AD_NONE;
    button_stable = BUTTON_AD_NONE;
    button_same_count = 0U;
    button_release_count = 0U;
    button_armed = 1U;
    readkey_last_raw = BUTTON_AD_NONE;
    readkey_stable = BUTTON_AD_NONE;
    readkey_same_count = 0U;
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
        if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
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

    if (raw == readkey_last_raw)
    {
        if (readkey_same_count < 255U)
        {
            readkey_same_count++;
        }
    }
    else
    {
        readkey_same_count = 0U;
        readkey_last_raw = raw;
    }

    if ((readkey_same_count >= BUTTON_AD_DEBOUNCE_COUNT) && (raw != readkey_stable))
    {
        readkey_stable = raw;
    }

    return readkey_stable;
}

ButtonAD_Key_t ButtonAD_GetEvent(void)
{
    ButtonAD_Key_t raw = ButtonAD_ReadKey();

    if (raw == button_last_raw)
    {
        if (button_same_count < 255U)
        {
            button_same_count++;
        }
    }
    else
    {
        button_same_count = 0U;
        button_last_raw = raw;
    }

    if ((button_same_count >= BUTTON_AD_DEBOUNCE_COUNT) && (raw != button_stable))
    {
        button_stable = raw;

        if (raw == BUTTON_AD_NONE)
        {
            if (button_release_count < 255U)
            {
                button_release_count++;
            }
            if (button_release_count >= BUTTON_AD_RELEASE_COUNT)
            {
                button_armed = 1U;
            }
        }
        else
        {
            button_release_count = 0U;
            if (button_armed)
            {
                button_armed = 0U;
                return raw;
            }
        }
    }

    return BUTTON_AD_NONE;
}
