#include "bad_apple.h"

#include "oled.h"
#include "bad_apple_frames.h"

#define BAD_APPLE_FRAME_MS 50U

static uint8_t g_frame_idx = 0U;
static uint32_t g_last_frame_ms = 0U;

static void BadApple_SetPos(uint8_t x, uint8_t page)
{
    /* Keep +2 column offset to match existing OLED driver behavior. */
    x += 2U;
    oled_write_byte((uint8_t)(0xB0U + page), OLED_CMD);
    oled_write_byte((uint8_t)(((x & 0xF0U) >> 4) | 0x10U), OLED_CMD);
    oled_write_byte((uint8_t)(x & 0x0FU), OLED_CMD);
}

static void BadApple_DrawRawFrame(const uint8_t *frame)
{
    uint8_t page = 0U;
    uint8_t x = 0U;

    for (page = 0U; page < 8U; page++)
    {
        BadApple_SetPos(0U, page);
        for (x = 0U; x < 128U; x++)
        {
            oled_write_byte((uint8_t)(~frame[(page * 128U) + x]), OLED_DATA);
        }
    }
}

void BadApple_Reset(void)
{
    g_frame_idx = 0U;
    g_last_frame_ms = 0U;
}

void BadApple_DrawFrame(uint32_t now_ms)
{
    if ((g_last_frame_ms == 0U) || ((now_ms - g_last_frame_ms) >= BAD_APPLE_FRAME_MS))
    {
        g_last_frame_ms = now_ms;
        g_frame_idx = (uint8_t)(g_frame_idx + 1U);
        if (g_frame_idx >= BAD_APPLE_FRAME_COUNT)
        {
            g_frame_idx = 0U;
        }
    }

    BadApple_DrawRawFrame(bad_apple_frames[g_frame_idx]);
}
