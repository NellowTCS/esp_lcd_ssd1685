/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: MIT
 *
 * esp_lcd_ssd1685.c
 *
 * Full-featured ESP-IDF LCD panel driver for Solomon Systech SSD1685
 * (pin/register compatible with SSD1680, SSD1681, SSD1683).
 *
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "esp_lcd_ssd1685.h"

static const char *TAG = "ssd1685";

/* -------------------------------------------------------------------------
 * Internal state structure
 * -------------------------------------------------------------------------*/
typedef struct {
    esp_lcd_panel_t        base;           /* MUST be first member           */
    esp_lcd_panel_io_handle_t io;

    /* Pin configuration */
    int                    reset_gpio;
    bool                   reset_level;    /* true = active high reset       */
    int                    busy_gpio;
    uint32_t               busy_timeout_ms;

    /* Panel geometry */
    uint16_t               width;          /* source pixels (horizontal)     */
    uint16_t               height;         /* gate lines    (vertical)       */

    /* Viewport offsets (from set_gap) */
    int                    x_gap;
    int                    y_gap;

    /* State flags */
    bool                   invert_color;
    bool                   swap_xy;
    bool                   mirror_x;
    bool                   mirror_y;
    bool                   non_copy_mode;  /* if true, draw_bitmap won't refresh */

    /* Active colour plane for draw_bitmap */
    ssd1685_color_plane_t  active_plane;

    /* Refresh mode used by auto-refresh inside draw_bitmap */
    ssd1685_refresh_mode_t refresh_mode;

    /* Optional custom LUT */
    const uint8_t         *custom_lut;
    size_t                 custom_lut_size;

    /* Optional reset callback */
    esp_err_t (*on_reset)(void *);
    void      *on_reset_user_data;
} ssd1685_panel_t;

/* -------------------------------------------------------------------------
 * Forward declarations of panel ops
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_del         (esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1685_reset       (esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1685_init        (esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1685_draw_bitmap (esp_lcd_panel_t *panel,
                                             int x_start, int y_start,
                                             int x_end,   int y_end,
                                             const void *color_data);
static esp_err_t panel_ssd1685_mirror      (esp_lcd_panel_t *panel,
                                             bool mx, bool my);
static esp_err_t panel_ssd1685_swap_xy     (esp_lcd_panel_t *panel,
                                             bool swap);
static esp_err_t panel_ssd1685_set_gap     (esp_lcd_panel_t *panel,
                                             int x_gap, int y_gap);
static esp_err_t panel_ssd1685_invert_color(esp_lcd_panel_t *panel,
                                             bool invert);
static esp_err_t panel_ssd1685_disp_on_off (esp_lcd_panel_t *panel, bool off);

/* -------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/* Send a command byte (D/C = 0) */
static inline esp_err_t epd_cmd(ssd1685_panel_t *s, uint8_t cmd)
{
    return esp_lcd_panel_io_tx_param(s->io, cmd, NULL, 0);
}

/* Send a command then data bytes */
static inline esp_err_t epd_cmd_data(ssd1685_panel_t *s, uint8_t cmd,
                                      const void *data, size_t len)
{
    return esp_lcd_panel_io_tx_param(s->io, cmd, data, len);
}

/* Send raw colour data (D/C = 1) - uses tx_color for potential DMA path */
static inline esp_err_t epd_color(ssd1685_panel_t *s,
                                   const void *data, size_t len)
{
    return esp_lcd_panel_io_tx_color(s->io, SSD1685_CMD_WRITE_RAM_BW, data, len);
}

/* -------------------------------------------------------------------------
 * Busy wait
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_wait_busy(esp_lcd_panel_handle_t handle)
{
    ssd1685_panel_t *s = __containerof(handle, ssd1685_panel_t, base);

    if (s->busy_gpio < 0) {
        /* No BUSY pin - use a worst-case delay */
        vTaskDelay(pdMS_TO_TICKS(500));
        return ESP_OK;
    }

    uint64_t start_us = esp_timer_get_time();
    uint64_t timeout_us = (uint64_t)s->busy_timeout_ms * 1000ULL;

    /* BUSY = HIGH means the controller is working.  Wait for it to go LOW. */
    while (gpio_get_level(s->busy_gpio) == 1) {
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            ESP_LOGE(TAG, "BUSY timeout after %lu ms", (unsigned long)s->busy_timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Compute data-entry-mode byte from mirror / swap flags
 *
 * SSD168x Data Entry Mode (0x11):
 *   Bit 2 : AM    - 0 = X direction, 1 = Y direction
 *   Bit 1 : ID[1] - Y increment direction
 *   Bit 0 : ID[0] - X increment direction
 *
 * We use X-first scan (AM=0) and map mirror/swap to the ID bits so that
 * the host co-ordinate system always has (0,0) at top-left.
 * -------------------------------------------------------------------------*/
static uint8_t compute_entry_mode(bool swap_xy, bool mirror_x, bool mirror_y)
{
    /* Default (no transforms): X increment, Y increment, X-first
     *   AM=0, ID[1]=1, ID[0]=1  → 0x03  */
    uint8_t mode = 0x03;

    if (swap_xy) {
        /* Switch to Y-first scan */
        mode = (mode & ~0x04) | 0x04;   /* AM = 1 */
        /* Swap the meaning of X/Y increment bits */
        bool old_id0 = (mode >> 0) & 1;
        bool old_id1 = (mode >> 1) & 1;
        mode = (mode & ~0x03) | ((uint8_t)old_id0 << 1) | (uint8_t)old_id1;
    }

    if (mirror_x) {
        mode ^= 0x01;   /* toggle ID[0] - X direction */
    }
    if (mirror_y) {
        mode ^= 0x02;   /* toggle ID[1] - Y direction */
    }

    return mode;
}

static esp_err_t update_entry_mode(ssd1685_panel_t *s)
{
    /* SSD168x does not provide a true 90-degree rotation. We keep the
     * hardware entry mode in X-first order and apply swap_xy in software. */
    bool hw_swap = false;
    bool hw_mirror_x = s->swap_xy ? false : s->mirror_x;
    bool hw_mirror_y = s->swap_xy ? false : s->mirror_y;
    uint8_t mode = compute_entry_mode(hw_swap, hw_mirror_x, hw_mirror_y);
    return epd_cmd_data(s, SSD1685_CMD_DATA_ENTRY_MODE, &mode, 1);
}

static inline void map_logical_to_physical(const ssd1685_panel_t *s,
                                           int lx, int ly,
                                           int *px, int *py)
{
    int x = lx;
    int y = ly;

    if (s->swap_xy) {
        int tmp = x;
        x = y;
        y = tmp;
    }
    if (s->mirror_x) {
        x = (int)s->width - 1 - x;
    }
    if (s->mirror_y) {
        y = (int)s->height - 1 - y;
    }

    *px = x;
    *py = y;
}

static inline uint8_t get_bit_1bpp(const uint8_t *src, int row_bytes, int x, int y)
{
    int idx = y * row_bytes + (x >> 3);
    uint8_t mask = (uint8_t)(1U << (7 - (x & 7)));
    return (src[idx] & mask) ? 1U : 0U;
}

static inline void set_bit_1bpp(uint8_t *dst, int row_bytes, int x, int y, uint8_t bit)
{
    int idx = y * row_bytes + (x >> 3);
    uint8_t mask = (uint8_t)(1U << (7 - (x & 7)));
    if (bit) {
        dst[idx] |= mask;
    } else {
        dst[idx] &= (uint8_t)~mask;
    }
}

/* Set RAM window and address counters so writes map to [x0,y0]→[x1-1,y1-1] */
static esp_err_t set_ram_window(ssd1685_panel_t *s,
                                 int x0, int y0, int x1, int y1)
{
    uint8_t buf[4];

    /* X address: in bytes (8 pixels per byte) */
    int xs_byte = x0 / 8;
    int xe_byte = (x1 - 1) / 8;

    /* Set X window  [0x44] param: Xstart_byte, Xend_byte */
    buf[0] = (uint8_t)xs_byte;
    buf[1] = (uint8_t)xe_byte;
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_SET_RAM_X_ADDR, buf, 2),
                        TAG, "set X addr window");

    /* Set Y window  [0x45] param: Ystart_L, Ystart_H, Yend_L, Yend_H */
    buf[0] = (uint8_t)(y0 & 0xFF);
    buf[1] = (uint8_t)((y0 >> 8) & 0x01);
    buf[2] = (uint8_t)((y1 - 1) & 0xFF);
    buf[3] = (uint8_t)(((y1 - 1) >> 8) & 0x01);
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_SET_RAM_Y_ADDR, buf, 4),
                        TAG, "set Y addr window");

    /* Reset address counters [0x4E], [0x4F] */
    buf[0] = (uint8_t)xs_byte;
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_SET_RAM_X_COUNTER, buf, 1),
                        TAG, "set X counter");

    buf[0] = (uint8_t)(y0 & 0xFF);
    buf[1] = (uint8_t)((y0 >> 8) & 0x01);
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_SET_RAM_Y_COUNTER, buf, 2),
                        TAG, "set Y counter");

    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Refresh sequence
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_refresh(esp_lcd_panel_handle_t handle,
                                   ssd1685_refresh_mode_t mode)
{
    ssd1685_panel_t *s = __containerof(handle, ssd1685_panel_t, base);
    uint8_t ctrl2;

    switch (mode) {
    case SSD1685_REFRESH_PARTIAL:
        ctrl2 = SSD1685_DISP_CTRL2_PARTIAL_REFRESH;
        break;
    case SSD1685_REFRESH_FAST:
        /* Fast mode: skip initial display stage (reduces ghosting penalty) */
        ctrl2 = 0xC7;
        break;
    default: /* FULL */
        ctrl2 = SSD1685_DISP_CTRL2_FULL_REFRESH;
        break;
    }

    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &ctrl2, 1),
                        TAG, "disp ctrl2");
    ESP_RETURN_ON_ERROR(epd_cmd(s, SSD1685_CMD_MASTER_ACTIVATION),
                        TAG, "master activation");
    ESP_RETURN_ON_ERROR(esp_lcd_ssd1685_wait_busy(handle),
                        TAG, "wait busy after refresh");
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Colour plane selection
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_set_color_plane(esp_lcd_panel_handle_t handle,
                                           ssd1685_color_plane_t plane)
{
    ssd1685_panel_t *s = __containerof(handle, ssd1685_panel_t, base);
    s->active_plane = plane;
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Clear (fill both planes, then refresh)
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_clear(esp_lcd_panel_handle_t handle, uint8_t color_byte)
{
    ssd1685_panel_t *s = __containerof(handle, ssd1685_panel_t, base);

    size_t num_bytes = ((size_t)s->width * s->height) / 8;
    /* Fill in chunks to avoid large heap allocation */
    enum { CHUNK = 256 };
    uint8_t chunk[CHUNK];
    memset(chunk, color_byte, CHUNK);

    /* B/W plane */
    ESP_RETURN_ON_ERROR(set_ram_window(s, 0, 0, s->width, s->height),
                        TAG, "clear: bw window");
    /* Start pixel-data stream for B/W plane */
    /* esp_lcd_panel_io_tx_color sends the command byte first */
    size_t remaining = num_bytes;
    /* We can't stream with tx_color in multiple calls easily, so we
     * set the window and use tx_param for the first chunk, tx_color for rest.
     * Actually the simplest approach: allocate heap if small enough, otherwise
     * loop with window re-set (not needed since address auto-increments). */

    /* Use a looping approach: send command once, data in chunks via tx_color */
    {
        /* Send write-RAM command (this also sets D/C=1 for subsequent data) */
        /* First call with cmd sets up the write; subsequent calls need to be
         * raw data - but the IO layer wraps each call with cmd.  The SSD168x
         * auto-increments the RAM counter on each byte so we can break the
         * transfer into multiple esp_lcd_panel_io_tx_color() calls as long as
         * we don't interleave commands. */
        size_t first_chunk = remaining < CHUNK ? remaining : CHUNK;
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_io_tx_color(s->io, SSD1685_CMD_WRITE_RAM_BW,
                                      chunk, first_chunk),
            TAG, "clear bw first chunk");
        remaining -= first_chunk;

        while (remaining > 0) {
            size_t sz = remaining < CHUNK ? remaining : CHUNK;
            ESP_RETURN_ON_ERROR(
                esp_lcd_panel_io_tx_color(s->io, SSD1685_CMD_WRITE_RAM_BW,
                                          chunk, sz),
                TAG, "clear bw chunk");
            remaining -= sz;
        }
    }

    /* RED plane - fill with 0x00 (no red) */
    uint8_t red_fill = 0x00;
    uint8_t red_chunk[CHUNK];
    memset(red_chunk, red_fill, CHUNK);
    ESP_RETURN_ON_ERROR(set_ram_window(s, 0, 0, s->width, s->height),
                        TAG, "clear: red window");
    remaining = num_bytes;
    {
        size_t first_chunk = remaining < CHUNK ? remaining : CHUNK;
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_io_tx_color(s->io, SSD1685_CMD_WRITE_RAM_RED,
                                      red_chunk, first_chunk),
            TAG, "clear red first chunk");
        remaining -= first_chunk;
        while (remaining > 0) {
            size_t sz = remaining < CHUNK ? remaining : CHUNK;
            ESP_RETURN_ON_ERROR(
                esp_lcd_panel_io_tx_color(s->io, SSD1685_CMD_WRITE_RAM_RED,
                                          red_chunk, sz),
                TAG, "clear red chunk");
            remaining -= sz;
        }
    }

    /* Trigger full refresh */
    return esp_lcd_ssd1685_refresh(handle, SSD1685_REFRESH_FULL);
}

/* -------------------------------------------------------------------------
 * Deep sleep
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_sleep(esp_lcd_panel_handle_t handle, uint8_t mode)
{
    ssd1685_panel_t *s = __containerof(handle, ssd1685_panel_t, base);
    return epd_cmd_data(s, SSD1685_CMD_DEEP_SLEEP, &mode, 1);
}

/* -------------------------------------------------------------------------
 * Load custom LUT
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_load_lut(esp_lcd_panel_handle_t handle,
                                    const uint8_t *lut, size_t lut_size)
{
    ssd1685_panel_t *s = __containerof(handle, ssd1685_panel_t, base);
    ESP_RETURN_ON_FALSE(lut && lut_size > 0, ESP_ERR_INVALID_ARG, TAG, "bad LUT");

    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_WRITE_LUT, lut, lut_size),
                        TAG, "write LUT");

    /* Signal to load the new LUT then wait */
    uint8_t ctrl2 = 0x80;   /* Enable clock, load LUT */
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &ctrl2, 1),
                        TAG, "disp ctrl2 load lut");
    ESP_RETURN_ON_ERROR(epd_cmd(s, SSD1685_CMD_MASTER_ACTIVATION), TAG, "activate lut");
    ESP_RETURN_ON_ERROR(esp_lcd_ssd1685_wait_busy(handle), TAG, "wait lut busy");
    return ESP_OK;
}

/* =========================================================================
 * Factory function - esp_lcd_new_panel_ssd1685()
 * =========================================================================*/
esp_err_t esp_lcd_new_panel_ssd1685(
        const esp_lcd_panel_io_handle_t io,
        const esp_lcd_panel_dev_config_t *panel_dev_cfg,
        esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    ssd1685_panel_t *s = NULL;

    ESP_GOTO_ON_FALSE(io && panel_dev_cfg && ret_panel,
                      ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

    const esp_lcd_panel_ssd1685_config_t *cfg =
            (const esp_lcd_panel_ssd1685_config_t *)panel_dev_cfg->vendor_config;

    ESP_GOTO_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, err, TAG,
                      "vendor_config must point to esp_lcd_panel_ssd1685_config_t");

    s = (ssd1685_panel_t *)calloc(1, sizeof(ssd1685_panel_t));
    ESP_GOTO_ON_FALSE(s, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1685 panel");

    /* Populate internal state */
    s->io            = io;
    s->reset_gpio    = panel_dev_cfg->reset_gpio_num;
    s->reset_level   = panel_dev_cfg->flags.reset_active_high;
    s->busy_gpio     = cfg->busy_gpio_num;
    s->busy_timeout_ms = cfg->busy_timeout_ms ? cfg->busy_timeout_ms : 5000;
    s->width         = cfg->panel_width  ? cfg->panel_width  : 296;
    s->height        = cfg->panel_height ? cfg->panel_height : 128;
    s->non_copy_mode = cfg->non_copy_mode;
    s->refresh_mode  = cfg->default_refresh_mode;
    s->active_plane  = SSD1685_COLOR_PLANE_BW;
    s->custom_lut    = cfg->custom_lut;
    s->custom_lut_size = cfg->custom_lut_size;
    s->on_reset      = cfg->on_reset;
    s->on_reset_user_data = cfg->on_reset_user_data;

    /* Configure BUSY GPIO as input */
    if (s->busy_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << s->busy_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "BUSY gpio config");
    }

    /* Configure RST GPIO as output if given */
    if (s->reset_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << s->reset_gpio),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "RST gpio config");
    }

    /* Wire up the panel ops vtable */
    s->base.del          = panel_ssd1685_del;
    s->base.reset        = panel_ssd1685_reset;
    s->base.init         = panel_ssd1685_init;
    s->base.draw_bitmap  = panel_ssd1685_draw_bitmap;
    s->base.mirror       = panel_ssd1685_mirror;
    s->base.swap_xy      = panel_ssd1685_swap_xy;
    s->base.set_gap      = panel_ssd1685_set_gap;
    s->base.invert_color = panel_ssd1685_invert_color;
    s->base.disp_on_off  = panel_ssd1685_disp_on_off;

    *ret_panel = &s->base;
    ESP_LOGD(TAG, "new ssd1685 panel @%p  %dx%d", s, s->width, s->height);
    return ESP_OK;

err:
    if (s) {
        if (s->reset_gpio >= 0) gpio_reset_pin((gpio_num_t)s->reset_gpio);
        if (s->busy_gpio  >= 0) gpio_reset_pin((gpio_num_t)s->busy_gpio);
        free(s);
    }
    return ret;
}

/* =========================================================================
 * Panel ops implementations
 * =========================================================================*/

static esp_err_t panel_ssd1685_del(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    if (s->reset_gpio >= 0) gpio_reset_pin((gpio_num_t)s->reset_gpio);
    if (s->busy_gpio  >= 0) gpio_reset_pin((gpio_num_t)s->busy_gpio);
    ESP_LOGD(TAG, "del ssd1685 panel @%p", s);
    free(s);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * reset
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_reset(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);

    /* Optional pre-reset callback (e.g. power sequencing) */
    if (s->on_reset) {
        ESP_RETURN_ON_ERROR(s->on_reset(s->on_reset_user_data),
                            TAG, "on_reset callback");
    }

    if (s->reset_gpio >= 0) {
        /* Assert reset (active low by default) */
        gpio_set_level((gpio_num_t)s->reset_gpio, s->reset_level ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        /* De-assert */
        gpio_set_level((gpio_num_t)s->reset_gpio, s->reset_level ? 0 : 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Software reset and wait for panel to be ready */
    ESP_RETURN_ON_ERROR(epd_cmd(s, SSD1685_CMD_SW_RESET), TAG, "sw reset");
    ESP_RETURN_ON_ERROR(esp_lcd_ssd1685_wait_busy(panel), TAG, "wait after reset");

    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * init
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_init(esp_lcd_panel_t *panel)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    uint8_t buf[5];

    /* 1. Driver output control  (0x01)
     *    Sets total gate lines = (height - 1), scan direction TB=0 (G0→Gn)  */
    uint16_t gate = (uint16_t)(s->height - 1);
    buf[0] = (uint8_t)(gate & 0xFF);
    buf[1] = (uint8_t)((gate >> 8) & 0x01);
    buf[2] = 0x00;   /* GD=0, SM=0, TB=0 */
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_DRIVER_OUTPUT_CTRL, buf, 3),
                        TAG, "driver output ctrl");

    /* 2. Booster soft start  (0x0C)
     *    Common values from Waveshare / Good Display reference code */
    buf[0] = 0xAE; buf[1] = 0xC7; buf[2] = 0xC3; buf[3] = 0xC0; buf[4] = 0x40;
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_BOOST_SOFTSTART, buf, 5),
                        TAG, "boost softstart");

    /* 3. Data entry mode  (0x11) */
    ESP_RETURN_ON_ERROR(update_entry_mode(s), TAG, "data entry mode");

    /* 4. Set full RAM window */
    ESP_RETURN_ON_ERROR(set_ram_window(s, 0, 0, s->width, s->height),
                        TAG, "full RAM window");

    /* 5. Border waveform  (0x3C)  -  HIZ (float) border */
    buf[0] = 0x01;
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_WRITE_BORDER, buf, 1),
                        TAG, "border waveform");

    /* 6. Temperature sensor  (0x18)  - internal */
    buf[0] = SSD1685_TEMP_SENSOR_INTERNAL;
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_TEMP_SENSOR_CTRL, buf, 1),
                        TAG, "temp sensor");

    /* 7. VCOM  (0x2C)  - -1.5 V (0x36) is a safe default; panel-specific */
    buf[0] = 0x36;
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_WRITE_VCOM_REG, buf, 1),
                        TAG, "vcom");

    /* 8. Load OTP waveform: activate LUT from OTP */
    buf[0] = 0xB1;   /* enable clock + analog, load LUT from OTP */
    ESP_RETURN_ON_ERROR(epd_cmd_data(s, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, buf, 1),
                        TAG, "disp ctrl2 otp lut");
    ESP_RETURN_ON_ERROR(epd_cmd(s, SSD1685_CMD_MASTER_ACTIVATION), TAG, "activate otp");
    ESP_RETURN_ON_ERROR(esp_lcd_ssd1685_wait_busy(panel), TAG, "wait otp");

    /* 9. Optionally load custom LUT */
    if (s->custom_lut && s->custom_lut_size > 0) {
        ESP_RETURN_ON_ERROR(
            esp_lcd_ssd1685_load_lut(panel, s->custom_lut, s->custom_lut_size),
            TAG, "custom lut");
    }

    ESP_LOGD(TAG, "ssd1685 init OK  %dx%d", s->width, s->height);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * draw_bitmap
 *
 * Parameters use the standard esp_lcd convention:
 *   x_start, y_start  - inclusive top-left
 *   x_end,   y_end    - exclusive bottom-right
 *   color_data         - packed 1 bpp bitmap, 1 = white / 0 = black,
 *                        MSB first, rows padded to byte boundary
 *
 * The function:
 *   1. Applies x_gap / y_gap offsets
 *   2. Sets the RAM window to the target rectangle
 *   3. Streams the data, applying colour inversion if requested
 *   4. Unless non_copy_mode is set, triggers a display refresh
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_draw_bitmap(esp_lcd_panel_t *panel,
                                            int x_start, int y_start,
                                            int x_end,   int y_end,
                                            const void *color_data)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "null color_data");

    int logical_width = s->swap_xy ? s->height : s->width;
    int logical_height = s->swap_xy ? s->width : s->height;

    /* Apply viewport offset */
    x_start += s->x_gap;
    x_end   += s->x_gap;
    y_start += s->y_gap;
    y_end   += s->y_gap;

    /* Clamp to logical bounds */
    if (x_start < 0) x_start = 0;
    if (y_start < 0) y_start = 0;
    if (x_end > logical_width)  x_end  = logical_width;
    if (y_end > logical_height) y_end  = logical_height;
    if (x_start >= x_end || y_start >= y_end) {
        ESP_LOGW(TAG, "draw_bitmap: empty or out-of-bounds rect");
        return ESP_OK;
    }

    /* Determine which RAM plane to write */
    uint8_t write_cmd = (s->active_plane == SSD1685_COLOR_PLANE_RED)
                        ? SSD1685_CMD_WRITE_RAM_RED
                        : SSD1685_CMD_WRITE_RAM_BW;

    if (!s->swap_xy) {
        /* Set addressing window */
        ESP_RETURN_ON_ERROR(
            set_ram_window(s, x_start, y_start, x_end, y_end),
            TAG, "set RAM window");

        /* Row width in bytes - each row is (x_end - x_start) bits, rounded up */
        int row_pixels = x_end - x_start;
        int row_bytes  = (row_pixels + 7) / 8;
        int num_rows   = y_end - y_start;
        size_t total_bytes = (size_t)row_bytes * num_rows;

        if (!s->invert_color) {
            /* Fast path: send data as-is */
            ESP_RETURN_ON_ERROR(
                esp_lcd_panel_io_tx_color(s->io, write_cmd, color_data, total_bytes),
                TAG, "tx_color bw");
        } else {
            /* Invert path: process in 256-byte chunks on the stack */
            enum { CHUNK_SZ = 256 };
            uint8_t chunk[CHUNK_SZ];
            const uint8_t *src = (const uint8_t *)color_data;
            size_t remaining   = total_bytes;
            bool   first_chunk = true;

            while (remaining > 0) {
                size_t sz = remaining < CHUNK_SZ ? remaining : CHUNK_SZ;
                for (size_t i = 0; i < sz; i++) {
                    chunk[i] = (uint8_t)~src[i];
                }
                if (first_chunk) {
                    ESP_RETURN_ON_ERROR(
                        esp_lcd_panel_io_tx_color(s->io, write_cmd, chunk, sz),
                        TAG, "tx_color inv first");
                    first_chunk = false;
                } else {
                    /* Subsequent chunks: keep the same write command active by
                     * re-issuing tx_color.  The SSD168x auto-increments, so
                     * each call simply continues the stream. */
                    ESP_RETURN_ON_ERROR(
                        esp_lcd_panel_io_tx_color(s->io, write_cmd, chunk, sz),
                        TAG, "tx_color inv chunk");
                }
                src       += sz;
                remaining -= sz;
            }
        }
    } else {
        /* Software rotation path for swap_xy */
        int src_w = x_end - x_start;
        int src_h = y_end - y_start;

        int px0, py0, px1, py1, px2, py2, px3, py3;
        map_logical_to_physical(s, x_start,     y_start,     &px0, &py0);
        map_logical_to_physical(s, x_end - 1,   y_start,     &px1, &py1);
        map_logical_to_physical(s, x_start,     y_end - 1,   &px2, &py2);
        map_logical_to_physical(s, x_end - 1,   y_end - 1,   &px3, &py3);

        int x_min = px0;
        int x_max = px0;
        int y_min = py0;
        int y_max = py0;
        if (px1 < x_min) {
            x_min = px1;
        }
        if (px1 > x_max) {
            x_max = px1;
        }
        if (px2 < x_min) {
            x_min = px2;
        }
        if (px2 > x_max) {
            x_max = px2;
        }
        if (px3 < x_min) {
            x_min = px3;
        }
        if (px3 > x_max) {
            x_max = px3;
        }
        if (py1 < y_min) {
            y_min = py1;
        }
        if (py1 > y_max) {
            y_max = py1;
        }
        if (py2 < y_min) {
            y_min = py2;
        }
        if (py2 > y_max) {
            y_max = py2;
        }
        if (py3 < y_min) {
            y_min = py3;
        }
        if (py3 > y_max) {
            y_max = py3;
        }

        int dest_x0 = x_min;
        int dest_y0 = y_min;
        int dest_w = (x_max - x_min) + 1;
        int dest_h = (y_max - y_min) + 1;

        int src_row_bytes = (src_w + 7) / 8;
        int dest_row_bytes = (dest_w + 7) / 8;
        size_t dest_size = (size_t)dest_row_bytes * dest_h;

        uint8_t *rot = (uint8_t *)calloc(1, dest_size);
        ESP_RETURN_ON_FALSE(rot, ESP_ERR_NO_MEM, TAG, "rotate: no mem");

        const uint8_t *src = (const uint8_t *)color_data;
        for (int ly = 0; ly < src_h; ly++) {
            if ((ly & 0x1F) == 0) {
                esp_task_wdt_reset();
            }
            for (int lx = 0; lx < src_w; lx++) {
                uint8_t bit = get_bit_1bpp(src, src_row_bytes, lx, ly);
                if (s->invert_color) {
                    bit ^= 1U;
                }
                if (bit == 0) {
                    continue;
                }

                int px, py;
                map_logical_to_physical(s, x_start + lx, y_start + ly, &px, &py);
                int dx = px - dest_x0;
                int dy = py - dest_y0;
                set_bit_1bpp(rot, dest_row_bytes, dx, dy, 1U);
            }
        }

        ESP_RETURN_ON_ERROR(set_ram_window(s, dest_x0, dest_y0,
                                           dest_x0 + dest_w, dest_y0 + dest_h),
                            TAG, "set RAM window rot");
        esp_err_t tx_ret = esp_lcd_panel_io_tx_color(s->io, write_cmd, rot, dest_size);
        free(rot);
        ESP_RETURN_ON_ERROR(tx_ret, TAG, "tx_color rot");
    }

    /* Auto-refresh unless the caller wants to batch updates */
    if (!s->non_copy_mode) {
        ESP_RETURN_ON_ERROR(
            esp_lcd_ssd1685_refresh(panel, s->refresh_mode),
            TAG, "auto refresh");
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * mirror / swap_xy
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_mirror(esp_lcd_panel_t *panel, bool mx, bool my)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    s->mirror_x = mx;
    s->mirror_y = my;
    /* Apply immediately */
    return update_entry_mode(s);
}

static esp_err_t panel_ssd1685_swap_xy(esp_lcd_panel_t *panel, bool swap)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    s->swap_xy = swap;
    return update_entry_mode(s);
}

/* -------------------------------------------------------------------------
 * set_gap
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_set_gap(esp_lcd_panel_t *panel,
                                        int x_gap, int y_gap)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    s->x_gap = x_gap;
    s->y_gap = y_gap;
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * invert_color
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_invert_color(esp_lcd_panel_t *panel, bool invert)
{
    ssd1685_panel_t *s = __containerof(panel, ssd1685_panel_t, base);
    s->invert_color = invert;
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * disp_on_off
 *
 * There is no backlight on an EPD.  We map this to:
 *   off = true  → deep sleep (image retained, display shows last frame)
 *   off = false → wake: SW reset + init
 * -------------------------------------------------------------------------*/
static esp_err_t panel_ssd1685_disp_on_off(esp_lcd_panel_t *panel, bool off)
{
    if (off) {
        return esp_lcd_ssd1685_sleep(panel, SSD1685_DEEP_SLEEP_MODE1);
    } else {
        /* Re-initialise from scratch */
        ESP_RETURN_ON_ERROR(panel_ssd1685_reset(panel),  TAG, "wake: reset");
        ESP_RETURN_ON_ERROR(panel_ssd1685_init(panel),   TAG, "wake: init");
        return ESP_OK;
    }
}
