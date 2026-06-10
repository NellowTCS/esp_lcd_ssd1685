/*
 * esp_lcd_ssd1685: Non-LVGL demo
 *
 * Drives the SSD1685 (SSD168x) e-paper display directly via esp_lcd
 * without any graphics framework.  Draws a checkerboard, some stripes,
 * and a simple hand-crafted bitmap to prove the driver is working.
 *
 * Pin configuration is at the top: change to match your wiring.
 *
 * Tested with CL-32 (168 x 384 panel, landscape = 384 wide x 168 tall).
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"

#include "esp_lcd_ssd1685.h"

static const char *TAG = "epd_demo";

/* Pin / panel configuration: edit these to match your hardware */
#define DEMO_SPI_HOST SPI2_HOST
#define DEMO_PIN_MOSI 10
#define DEMO_PIN_MISO 11 /* not used by EPD, set to -1 if not wired */
#define DEMO_PIN_SCLK 9
#define DEMO_PIN_CS 6
#define DEMO_PIN_DC 13
#define DEMO_PIN_RST 12
#define DEMO_PIN_BUSY 14

/* Physical panel dimensions (source x gate) BEFORE any rotation.
 * For the 2.9" 168x384 panel on the CL-32:
 *   source = 168  (width in hardware terms)
 *   gate   = 384  (height in hardware terms)
 */
#define PANEL_WIDTH 168
#define PANEL_HEIGHT 384

/* Source-line offset on this panel variant.
 * The SSD168x RAM is wider than the connected panel sources.  GAP_X=8
 * means the first live source line is RAM column 8.
 * DRAW_WIDTH = PANEL_WIDTH - PANEL_GAP_X is the maximum safe bitmap width. 
 */
#define PANEL_GAP_X 8
#define PANEL_GAP_Y 0
#define DRAW_WIDTH (PANEL_WIDTH - PANEL_GAP_X)
#define DRAW_HEIGHT (PANEL_HEIGHT - PANEL_GAP_Y)

/* SPI clock, EPDs are typically happy at 4 MHz */
#define DEMO_SPI_CLK_HZ (4 * 1000 * 1000)

/* Bitmap helpers
 *
 * 1bpp, 1 = white, 0 = black.  LSB = leftmost pixel within each byte to
 * match the SSD1685 source-line mapping on this panel.
 * Row stride = (width + 7) / 8 bytes. 
 */

static inline void bmp_set_pixel(uint8_t *buf, int stride, int x, int y, int white)
{
    int byte_idx = y * stride + (x >> 3);
    uint8_t mask = (uint8_t)(1U << (x & 7));
    if (white)
        buf[byte_idx] |= mask;
    else
        buf[byte_idx] &= (uint8_t)~mask;
}

static void bmp_fill_rect(uint8_t *buf, int buf_w, int buf_h,
                          int x0, int y0, int w, int h, int white)
{
    int stride = (buf_w + 7) / 8;
    for (int y = y0; y < y0 + h && y < buf_h; y++)
        for (int x = x0; x < x0 + w && x < buf_w; x++)
            bmp_set_pixel(buf, stride, x, y, white);
}

static void bmp_draw_rect(uint8_t *buf, int buf_w, int buf_h,
                          int x0, int y0, int w, int h, int white)
{
    int stride = (buf_w + 7) / 8;
    for (int x = x0; x < x0 + w && x < buf_w; x++)
    {
        bmp_set_pixel(buf, stride, x, y0, white);
        bmp_set_pixel(buf, stride, x, y0 + h - 1, white);
    }
    for (int y = y0; y < y0 + h && y < buf_h; y++)
    {
        bmp_set_pixel(buf, stride, x0, y, white);
        bmp_set_pixel(buf, stride, x0 + w - 1, y, white);
    }
}

/* 8x8 public-domain bitmap font, ASCII 0x20-0x7F */
static const uint8_t font8x8[96][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* ' ' */
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, /* '!' */
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '"' */
    {0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00}, /* '#' */
    {0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00}, /* '$' */
    {0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00}, /* '%' */
    {0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00}, /* '&' */
    {0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}, /* ''' */
    {0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00}, /* '(' */
    {0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00}, /* ')' */
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00}, /* '*' */
    {0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00}, /* '+' */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06}, /* ',' */
    {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00}, /* '-' */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00}, /* '.' */
    {0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00}, /* '/' */
    {0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00}, /* '0' */
    {0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00}, /* '1' */
    {0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00}, /* '2' */
    {0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00}, /* '3' */
    {0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00}, /* '4' */
    {0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00}, /* '5' */
    {0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00}, /* '6' */
    {0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00}, /* '7' */
    {0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00}, /* '8' */
    {0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00}, /* '9' */
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00}, /* ':' */
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06}, /* ';' */
    {0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00}, /* '<' */
    {0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00}, /* '=' */
    {0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00}, /* '>' */
    {0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00}, /* '?' */
    {0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00}, /* '@' */
    {0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00}, /* 'A' */
    {0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00}, /* 'B' */
    {0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00}, /* 'C' */
    {0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00}, /* 'D' */
    {0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00}, /* 'E' */
    {0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00}, /* 'F' */
    {0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00}, /* 'G' */
    {0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00}, /* 'H' */
    {0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, /* 'I' */
    {0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00}, /* 'J' */
    {0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00}, /* 'K' */
    {0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00}, /* 'L' */
    {0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00}, /* 'M' */
    {0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00}, /* 'N' */
    {0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00}, /* 'O' */
    {0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00}, /* 'P' */
    {0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00}, /* 'Q' */
    {0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00}, /* 'R' */
    {0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00}, /* 'S' */
    {0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, /* 'T' */
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00}, /* 'U' */
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00}, /* 'V' */
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}, /* 'W' */
    {0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00}, /* 'X' */
    {0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00}, /* 'Y' */
    {0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00}, /* 'Z' */
    {0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00}, /* '[' */
    {0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00}, /* '\\' */
    {0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00}, /* ']' */
    {0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00}, /* '^' */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}, /* '_' */
    {0x0C, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '`' */
    {0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00}, /* 'a' */
    {0x07, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3B, 0x00}, /* 'b' */
    {0x00, 0x00, 0x1E, 0x33, 0x03, 0x33, 0x1E, 0x00}, /* 'c' */
    {0x38, 0x30, 0x30, 0x3E, 0x33, 0x33, 0x6E, 0x00}, /* 'd' */
    {0x00, 0x00, 0x1E, 0x33, 0x3F, 0x03, 0x1E, 0x00}, /* 'e' */
    {0x1C, 0x36, 0x06, 0x0F, 0x06, 0x06, 0x0F, 0x00}, /* 'f' */
    {0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x1F}, /* 'g' */
    {0x07, 0x06, 0x36, 0x6E, 0x66, 0x66, 0x67, 0x00}, /* 'h' */
    {0x0C, 0x00, 0x0E, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, /* 'i' */
    {0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E}, /* 'j' */
    {0x07, 0x06, 0x66, 0x36, 0x1E, 0x36, 0x67, 0x00}, /* 'k' */
    {0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, /* 'l' */
    {0x00, 0x00, 0x33, 0x7F, 0x7F, 0x6B, 0x63, 0x00}, /* 'm' */
    {0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x33, 0x00}, /* 'n' */
    {0x00, 0x00, 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x00}, /* 'o' */
    {0x00, 0x00, 0x3B, 0x66, 0x66, 0x3E, 0x06, 0x0F}, /* 'p' */
    {0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x78}, /* 'q' */
    {0x00, 0x00, 0x3B, 0x6E, 0x66, 0x06, 0x0F, 0x00}, /* 'r' */
    {0x00, 0x00, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x00}, /* 's' */
    {0x08, 0x0C, 0x3E, 0x0C, 0x0C, 0x2C, 0x18, 0x00}, /* 't' */
    {0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6E, 0x00}, /* 'u' */
    {0x00, 0x00, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00}, /* 'v' */
    {0x00, 0x00, 0x63, 0x6B, 0x7F, 0x7F, 0x36, 0x00}, /* 'w' */
    {0x00, 0x00, 0x63, 0x36, 0x1C, 0x36, 0x63, 0x00}, /* 'x' */
    {0x00, 0x00, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x1F}, /* 'y' */
    {0x00, 0x00, 0x3F, 0x19, 0x0C, 0x26, 0x3F, 0x00}, /* 'z' */
    {0x38, 0x0C, 0x0C, 0x07, 0x0C, 0x0C, 0x38, 0x00}, /* '{' */
    {0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00}, /* '|' */
    {0x07, 0x0C, 0x0C, 0x38, 0x0C, 0x0C, 0x07, 0x00}, /* '}' */
    {0x6E, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* '~' */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* DEL / filled block */
};

static void bmp_draw_char(uint8_t *buf, int buf_w, int buf_h,
                          int x, int y, char c, int fg_white)
{
    int stride = (buf_w + 7) / 8;
    int idx = (unsigned char)c - 32;
    if (idx < 0 || idx >= 96)
        idx = 0;
    const uint8_t *glyph = font8x8[idx];
    for (int row = 0; row < 8 && (y + row) < buf_h; row++)
    {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8 && (x + col) < buf_w; col++)
        {
            int white = ((bits >> (7 - col)) & 1) ? fg_white : !fg_white;
            bmp_set_pixel(buf, stride, x + col, y + row, white);
        }
    }
}

static void bmp_draw_string(uint8_t *buf, int buf_w, int buf_h,
                            int x, int y, const char *str, int fg_white)
{
    while (*str)
    {
        if (x + 8 > buf_w)
            break;
        bmp_draw_char(buf, buf_w, buf_h, x, y, *str++, fg_white);
        x += 8;
    }
}

/* Demo scenes - all use w=DRAW_WIDTH, h=DRAW_HEIGHT */

static void demo_solid(esp_lcd_panel_handle_t panel, int w, int h)
{
    ESP_LOGI(TAG, "=== Scene 1: solid white / solid black ===");

    /* esp_lcd_ssd1685_clear fills the full RAM window itself */
    esp_lcd_ssd1685_clear(panel, 0xFF); /* white, also clears RED plane */
    vTaskDelay(pdMS_TO_TICKS(3000));

    size_t bytes = (size_t)((w + 7) / 8) * h;
    uint8_t *buf = malloc(bytes);
    if (!buf)
    {
        ESP_LOGE(TAG, "OOM");
        return;
    }
    memset(buf, 0x00, bytes); /* all black */
    esp_lcd_panel_draw_bitmap(panel, 0, 0, w, h, buf);
    esp_lcd_ssd1685_refresh(panel, SSD1685_REFRESH_FULL);
    free(buf);

    vTaskDelay(pdMS_TO_TICKS(3000));
}

static void demo_stripes(esp_lcd_panel_handle_t panel, int w, int h)
{
    ESP_LOGI(TAG, "=== Scene 2: vertical stripes ===");

    int stride = (w + 7) / 8;
    size_t bytes = (size_t)stride * h;
    uint8_t *buf = malloc(bytes);
    if (!buf)
    {
        ESP_LOGE(TAG, "OOM");
        return;
    }

    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            bmp_set_pixel(buf, stride, x, y, ((x / 16) & 1));

    esp_lcd_panel_draw_bitmap(panel, 0, 0, w, h, buf);
    esp_lcd_ssd1685_refresh(panel, SSD1685_REFRESH_FULL);
    free(buf);

    vTaskDelay(pdMS_TO_TICKS(4000));
}

static void demo_checker(esp_lcd_panel_handle_t panel, int w, int h)
{
    ESP_LOGI(TAG, "=== Scene 3: checkerboard ===");

    int stride = (w + 7) / 8;
    size_t bytes = (size_t)stride * h;
    uint8_t *buf = malloc(bytes);
    if (!buf)
    {
        ESP_LOGE(TAG, "OOM");
        return;
    }

    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            bmp_set_pixel(buf, stride, x, y, (((x / 8) + (y / 8)) & 1));

    esp_lcd_panel_draw_bitmap(panel, 0, 0, w, h, buf);
    esp_lcd_ssd1685_refresh(panel, SSD1685_REFRESH_FULL);
    free(buf);

    vTaskDelay(pdMS_TO_TICKS(4000));
}

static void demo_text(esp_lcd_panel_handle_t panel, int w, int h)
{
    ESP_LOGI(TAG, "=== Scene 4: text / borders ===");

    int stride = (w + 7) / 8;
    size_t bytes = (size_t)stride * h;
    uint8_t *buf = malloc(bytes);
    if (!buf)
    {
        ESP_LOGE(TAG, "OOM");
        return;
    }

    memset(buf, 0xFF, bytes); /* white background */

    bmp_draw_rect(buf, w, h, 0, 0, w, h, 0);      /* outer border */
    bmp_fill_rect(buf, w, h, 2, 2, w - 4, 18, 0); /* title bar */
    bmp_draw_string(buf, w, h, 4, 5, "SSD1685 Driver Demo", 1);

    char line[64];
    snprintf(line, sizeof(line), "Draw area: %d x %d px", w, h);
    bmp_draw_string(buf, w, h, 4, 26, line, 0);
    snprintf(line, sizeof(line), "Panel:     %d x %d  gap=(%d,%d)",
             PANEL_WIDTH, PANEL_HEIGHT, PANEL_GAP_X, PANEL_GAP_Y);
    bmp_draw_string(buf, w, h, 4, 36, line, 0);
    bmp_draw_string(buf, w, h, 4, 46, "1 bpp  MSB-first", 0);
    bmp_draw_string(buf, w, h, 4, 56, "esp_lcd_ssd1685 OK", 0);

    for (int i = 0; i < 8; i++)
        bmp_fill_rect(buf, w, h, 4 + i * 18, 70, 14, 14, (i & 1));

    bmp_draw_string(buf, w, h, 4, 90, "0123456789 ABCDEFGHIJKLM", 0);
    bmp_draw_string(buf, w, h, 4, 100, "abcdefghijklmnopqrstuvwx", 0);
    bmp_draw_string(buf, w, h, 4, 110, "!@#$%^&*()-+=[]{}|;:',./", 0);

    /* Diagonal to confirm square pixels */
    {
        int n = (w < h - 125) ? w : h - 125;
        for (int i = 0; i < n; i++)
            bmp_set_pixel(buf, stride, i, 125 + i, 0);
    }

    esp_lcd_panel_draw_bitmap(panel, 0, 0, w, h, buf);
    esp_lcd_ssd1685_refresh(panel, SSD1685_REFRESH_FULL);
    free(buf);

    vTaskDelay(pdMS_TO_TICKS(6000));
}

static void demo_partial(esp_lcd_panel_handle_t panel, int w, int h)
{
    ESP_LOGI(TAG, "=== Scene 5: partial area update ===");

    esp_lcd_ssd1685_clear(panel, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 64x64 centred square */
    int pw = 64, ph = 64;
    int px = (w - pw) / 2;
    int py = (h - ph) / 2;
    int pstride = (pw + 7) / 8;
    uint8_t *pbuf = malloc((size_t)pstride * ph);
    if (!pbuf)
        return;

    memset(pbuf, 0x00, (size_t)pstride * ph);     /* black fill */
    bmp_draw_rect(pbuf, pw, ph, 0, 0, pw, ph, 1); /* white border */

    esp_lcd_panel_draw_bitmap(panel, px, py, px + pw, py + ph, pbuf);
    esp_lcd_ssd1685_refresh(panel, SSD1685_REFRESH_FULL);
    free(pbuf);

    vTaskDelay(pdMS_TO_TICKS(4000));
}

/* app_main */
#ifdef __cplusplus
extern "C"
#endif
    void app_main(void)
{
    ESP_LOGI(TAG, "SSD1685 non-LVGL demo  panel=%dx%d  gap=(%d,%d)  draw=%dx%d",
             PANEL_WIDTH, PANEL_HEIGHT, PANEL_GAP_X, PANEL_GAP_Y,
             DRAW_WIDTH, DRAW_HEIGHT);

    /* 1. SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = DEMO_PIN_MOSI,
        .miso_io_num = DEMO_PIN_MISO,
        .sclk_io_num = DEMO_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (DRAW_WIDTH * DRAW_HEIGHT) / 8 + 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(DEMO_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* Panel IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = DEMO_PIN_CS,
        .dc_gpio_num = DEMO_PIN_DC,
        .spi_mode = 0,
        .pclk_hz = DEMO_SPI_CLK_HZ,
        .trans_queue_depth = 4,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {.sio_mode = 1},
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)DEMO_SPI_HOST, &io_cfg, &io_handle));

    /* SSD1685 config */
    esp_lcd_panel_ssd1685_config_t epd_cfg = {
        .busy_gpio_num = DEMO_PIN_BUSY,
        .busy_timeout_ms = 10000,
        .panel_width = PANEL_WIDTH,
        .panel_height = PANEL_HEIGHT,
        .non_copy_mode = false, /* auto-refresh after draw_bitmap */
        .default_refresh_mode = SSD1685_REFRESH_FULL,
        .custom_lut = NULL,
        .custom_lut_size = 0,
        .on_reset = NULL,
        .on_reset_user_data = NULL,
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = DEMO_PIN_RST,
        .bits_per_pixel = 1,
        .vendor_config = &epd_cfg,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1685(io_handle, &panel_cfg, &panel));

    /* Reset + init */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));

    /* Gap offsets the RAM window so pixel (0,0) maps to the first
     *    live source line.  Bitmaps must be at most DRAW_WIDTH wide. */
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, PANEL_GAP_X, PANEL_GAP_Y));

    ESP_LOGI(TAG, "Initial clear...");
    ESP_ERROR_CHECK(esp_lcd_ssd1685_clear(panel, 0xFF));
    ESP_LOGI(TAG, "Ready");

    while (true)
    {
        demo_solid(panel, DRAW_WIDTH, DRAW_HEIGHT);
        demo_stripes(panel, DRAW_WIDTH, DRAW_HEIGHT);
        demo_checker(panel, DRAW_WIDTH, DRAW_HEIGHT);
        demo_text(panel, DRAW_WIDTH, DRAW_HEIGHT);
        demo_partial(panel, DRAW_WIDTH, DRAW_HEIGHT);
    }
}