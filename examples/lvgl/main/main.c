/*
 * esp_lcd_ssd1685: LVGL demo
 *
 * Integrates the SSD1685 e-paper driver with LVGL 9.x using
 * LV_COLOR_FORMAT_I1 + LV_DISPLAY_RENDER_MODE_FULL.
 *
 * Pin configuration is at the top, change to match your wiring.
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "lvgl.h"
#include "esp_lcd_ssd1685.h"

static const char *TAG = "epd_lvgl";

/* Pin / panel configuration: edit these to match your hardware */
#define DEMO_SPI_HOST       SPI2_HOST
#define DEMO_PIN_MOSI       10
#define DEMO_PIN_MISO       11   /* -1 if not wired */
#define DEMO_PIN_SCLK       9
#define DEMO_PIN_CS         6
#define DEMO_PIN_DC         13
#define DEMO_PIN_RST        12
#define DEMO_PIN_BUSY       14

/* Physical panel dimensions (source x gate) BEFORE rotation.
 * For the 2.9" 168x384 CL-32 panel:                                      */
#define PANEL_WIDTH         168
#define PANEL_HEIGHT        384
#define PANEL_GAP_X         8
#define PANEL_GAP_Y         0

/* Rotation:  0 = portrait  (168 wide × 384 tall)
 *            1 = landscape (384 wide × 168 tall): most natural for CL-32 */
#define PANEL_ROTATION      1

/* SPI clock */
#define DEMO_SPI_CLK_HZ     (4 * 1000 * 1000)

/* LVGL tick period in ms */
#define LVGL_TICK_PERIOD_MS 5

/* Derived dimensions after rotation */
#if PANEL_ROTATION == 1 || PANEL_ROTATION == 3
#  define LVGL_WIDTH   PANEL_HEIGHT
#  define LVGL_HEIGHT  PANEL_WIDTH
#else
#  define LVGL_WIDTH   PANEL_WIDTH
#  define LVGL_HEIGHT  PANEL_HEIGHT
#endif

/* Global handles */
static esp_lcd_panel_handle_t  s_panel     = NULL;
static lv_display_t           *s_disp      = NULL;
static SemaphoreHandle_t       s_lvgl_mux  = NULL;

/* Pre-allocated repack buffer (EPD stride, no LVGL padding) */
static uint8_t *s_epd_buf = NULL;

/* Refresh semaphore + task, keeps the slow EPD refresh off the LVGL task */
static SemaphoreHandle_t s_refresh_sem  = NULL;
static TaskHandle_t      s_refresh_task = NULL;

/* EPD refresh task */
static void epd_refresh_task(void *arg)
{
    while (true) {
        if (xSemaphoreTake(s_refresh_sem, portMAX_DELAY) == pdTRUE) {
            esp_lcd_ssd1685_refresh(s_panel, SSD1685_REFRESH_FULL);
        }
    }
}

/* LVGL flush callback
 *
 * LVGL hands us a buffer whose rows are padded to 4 bytes.
 * We repack to 1-byte-aligned rows before calling draw_bitmap. */
static void lvgl_flush_cb(lv_display_t *disp,
                           const lv_area_t *area,
                           uint8_t *px_map)
{
    /* Area in the LVGL logical co-ordinate space */
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;
    int w  = x2 - x1 + 1;
    int h  = y2 - y1 + 1;

    /* LVGL I1 row stride (4-byte aligned) */
    int lvgl_stride = (int)(((unsigned int)(w + 31) / 32u) * 4u);

    /* EPD row stride (1-byte aligned) */
    int epd_stride  = (w + 7) / 8;

    /*
     * LVGL with LV_DISPLAY_RENDER_MODE_FULL always flushes the entire
     * framebuffer in a single call, so area == {0,0,w-1,h-1} and
     * lvgl_stride is based on the display width, not the area width.
     * Use the display width for the source stride.
     */
    int disp_w = (int)lv_display_get_horizontal_resolution(disp);
    int src_stride = (int)(((unsigned int)(disp_w + 31) / 32u) * 4u);

    if (lvgl_stride != epd_stride) {
        /* Repack: copy only the valid pixel bytes from each LVGL row */
        for (int row = 0; row < h; row++) {
            const uint8_t *src = px_map + (size_t)row * src_stride;
            uint8_t       *dst = s_epd_buf + (size_t)row * epd_stride;
            memcpy(dst, src, (size_t)epd_stride);
        }
        esp_lcd_panel_draw_bitmap(s_panel, x1, y1, x2 + 1, y2 + 1, s_epd_buf);
    } else {
        /* Strides match, pass the LVGL buffer directly */
        esp_lcd_panel_draw_bitmap(s_panel, x1, y1, x2 + 1, y2 + 1, px_map);
    }

    /* Signal the refresh task on the last flush of this render cycle */
    if (lv_display_flush_is_last(disp)) {
        xSemaphoreGive(s_refresh_sem);
    }

    lv_display_flush_ready(disp);
}

/* LVGL tick timer */
static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

/* LVGL task */
static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");

    while (true) {
        if (xSemaphoreTake(s_lvgl_mux, portMAX_DELAY)) {
            uint32_t ms_to_next = lv_timer_handler();
            xSemaphoreGive(s_lvgl_mux);
            if (ms_to_next == 0) ms_to_next = 1;
            vTaskDelay(pdMS_TO_TICKS(ms_to_next));
        }
    }
}

/* Demo UI
 *
 * Creates a simple LVGL screen to exercise the display. */
static void build_demo_ui(void)
{
    if (xSemaphoreTake(s_lvgl_mux, portMAX_DELAY) != pdTRUE) return;

    lv_obj_t *scr = lv_screen_active();

    /* Dark background */
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);

    /* Outer border */
    lv_obj_t *border = lv_obj_create(scr);
    lv_obj_set_size(border, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_opa(border, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(border, lv_color_white(), 0);
    lv_obj_set_style_border_width(border, 2, 0);
    lv_obj_set_style_radius(border, 0, 0);
    lv_obj_set_style_pad_all(border, 0, 0);
    lv_obj_center(border);

    /* Title label */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "SSD1685 LVGL Demo");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    /* Horizontal divider */
    lv_obj_t *line_obj = lv_obj_create(scr);
    lv_obj_set_size(line_obj, lv_pct(95), 1);
    lv_obj_set_style_bg_color(line_obj, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(line_obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(line_obj, 0, 0);
    lv_obj_align(line_obj, LV_ALIGN_TOP_MID, 0, 22);

    /* Info labels */
    char buf[80];
    snprintf(buf, sizeof(buf), "Resolution: %d x %d", LVGL_WIDTH, LVGL_HEIGHT);
    lv_obj_t *info1 = lv_label_create(scr);
    lv_label_set_text(info1, buf);
    lv_obj_set_style_text_color(info1, lv_color_white(), 0);
    lv_obj_align(info1, LV_ALIGN_TOP_LEFT, 8, 30);

    snprintf(buf, sizeof(buf), "Rotation: %d (hw swap+mirror)", PANEL_ROTATION);
    lv_obj_t *info2 = lv_label_create(scr);
    lv_label_set_text(info2, buf);
    lv_obj_set_style_text_color(info2, lv_color_white(), 0);
    lv_obj_align(info2, LV_ALIGN_TOP_LEFT, 8, 44);

    lv_obj_t *info3 = lv_label_create(scr);
    lv_label_set_text(info3, "Format: I1  Mode: FULL  Async refresh");
    lv_obj_set_style_text_color(info3, lv_color_white(), 0);
    lv_obj_align(info3, LV_ALIGN_TOP_LEFT, 8, 58);

    /* Checkerboard row of squares */
    int sq = 12;
    int count = (LVGL_WIDTH - 16) / (sq + 2);
    for (int i = 0; i < count; i++) {
        lv_obj_t *sq_obj = lv_obj_create(scr);
        lv_obj_set_size(sq_obj, sq, sq);
        lv_obj_set_style_border_width(sq_obj, 1, 0);
        lv_obj_set_style_border_color(sq_obj, lv_color_white(), 0);
        lv_obj_set_style_radius(sq_obj, 0, 0);
        if (i & 1) {
            lv_obj_set_style_bg_color(sq_obj, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(sq_obj, LV_OPA_COVER, 0);
        } else {
            lv_obj_set_style_bg_color(sq_obj, lv_color_black(), 0);
            lv_obj_set_style_bg_opa(sq_obj, LV_OPA_COVER, 0);
        }
        lv_obj_set_pos(sq_obj, 8 + i * (sq + 2), LVGL_HEIGHT / 2 - sq / 2);
    }

    /* Status bar at the bottom */
    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, "Driver: esp_lcd_ssd1685  |  OK");
    lv_obj_set_style_text_color(status, lv_color_white(), 0);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -6);

    xSemaphoreGive(s_lvgl_mux);

    ESP_LOGI(TAG, "Demo UI built, waiting for flush...");
}

/* app_main */
#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    ESP_LOGI(TAG, "SSD1685 LVGL demo starting...");

    /* 1. SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = DEMO_PIN_MOSI,
        .miso_io_num   = DEMO_PIN_MISO,
        .sclk_io_num   = DEMO_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (PANEL_WIDTH * PANEL_HEIGHT) / 8 + 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(DEMO_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 2. Panel IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = DEMO_PIN_CS,
        .dc_gpio_num       = DEMO_PIN_DC,
        .spi_mode          = 0,
        .pclk_hz           = DEMO_SPI_CLK_HZ,
        .trans_queue_depth = 4,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .flags = { .sio_mode = 1 },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)DEMO_SPI_HOST, &io_cfg, &io_handle));

    /* 3. SSD1685 config */
    esp_lcd_panel_ssd1685_config_t epd_cfg = {
        .busy_gpio_num        = DEMO_PIN_BUSY,
        .busy_timeout_ms      = 10000,
        .panel_width          = PANEL_WIDTH,
        .panel_height         = PANEL_HEIGHT,
        .non_copy_mode        = true,   /* We trigger refresh manually */
        .default_refresh_mode = SSD1685_REFRESH_FULL,
        .custom_lut           = NULL,
        .custom_lut_size      = 0,
        .on_reset             = NULL,
        .on_reset_user_data   = NULL,
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = DEMO_PIN_RST,
        .bits_per_pixel = 1,
        .vendor_config  = &epd_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1685(io_handle, &panel_cfg, &s_panel));

    /* 4. Reset + init */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    /* 5. Gap */
    if (PANEL_GAP_X || PANEL_GAP_Y)
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, PANEL_GAP_X, PANEL_GAP_Y));

    /* 6. Rotation (hardware: swap_xy + mirror flags)
     *    These must be set BEFORE LVGL is told the dimensions so the
     *    physical scan order matches what LVGL expects.
     *
     *    rot 0: portrait  - natural orientation, no transforms
     *    rot 1: landscape - swap_xy + mirror_x  (CW 90°)
     *    rot 2: 180°      - mirror_x + mirror_y
     *    rot 3: landscape - swap_xy + mirror_y  (CCW 90°)           */
#if PANEL_ROTATION == 1
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));
#elif PANEL_ROTATION == 2
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, true));
#elif PANEL_ROTATION == 3
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, true));
#endif

    /* 7. Initial clear */
    ESP_LOGI(TAG, "Initial clear...");
    ESP_ERROR_CHECK(esp_lcd_ssd1685_clear(s_panel, 0xFF));
    ESP_LOGI(TAG, "Clear done");

    /* 8. Allocate EPD repack buffer (1-byte-aligned rows) */
    size_t epd_stride = ((size_t)LVGL_WIDTH + 7u) / 8u;
    size_t epd_buf_sz = epd_stride * (size_t)LVGL_HEIGHT;
    s_epd_buf = heap_caps_malloc(epd_buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_epd_buf)
        s_epd_buf = heap_caps_malloc(epd_buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_ERROR_CHECK(s_epd_buf ? ESP_OK : ESP_ERR_NO_MEM);

    /* 9. Refresh task */
    s_refresh_sem = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_refresh_sem ? ESP_OK : ESP_ERR_NO_MEM);
    xTaskCreate(epd_refresh_task, "epdRefresh", 4096, NULL,
                tskIDLE_PRIORITY + 2, &s_refresh_task);

    /* 10. LVGL init */
    lv_init();

    /* LVGL tick timer */
    const esp_timer_create_args_t tick_timer_args = {
        .callback        = lvgl_tick_cb,
        .name            = "lvgl_tick",
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    /* 11. LVGL display
     *
     * LVGL I1 row stride (4-byte aligned):
     *   lvgl_stride = ((LVGL_WIDTH + 31) / 32) * 4
     *
     * We allocate two full-frame I1 buffers at LVGL's stride so LVGL
     * can use them without any internal realloc.                       */
    size_t lvgl_stride = (((size_t)LVGL_WIDTH + 31u) / 32u) * 4u;
    size_t lvgl_buf_sz = lvgl_stride * (size_t)LVGL_HEIGHT;

    ESP_LOGI(TAG, "LVGL  %dx%d  lvgl_stride=%zu  epd_stride=%zu  buf=%zu bytes",
             LVGL_WIDTH, LVGL_HEIGHT, lvgl_stride, epd_stride, lvgl_buf_sz);

    uint8_t *buf1 = heap_caps_malloc(lvgl_buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1) buf1 = heap_caps_malloc(lvgl_buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *buf2 = heap_caps_malloc(lvgl_buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf2) buf2 = heap_caps_malloc(lvgl_buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_ERROR_CHECK((buf1 && buf2) ? ESP_OK : ESP_ERR_NO_MEM);
    memset(buf1, 0xFF, lvgl_buf_sz);
    memset(buf2, 0xFF, lvgl_buf_sz);

    s_disp = lv_display_create(LVGL_WIDTH, LVGL_HEIGHT);
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(s_disp, buf1, buf2, lvgl_buf_sz,
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(s_disp, lvgl_flush_cb);

    /* 12. LVGL mutex + task */
    s_lvgl_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_lvgl_mux ? ESP_OK : ESP_ERR_NO_MEM);

    xTaskCreate(lvgl_task, "lvgl", 8192, NULL, tskIDLE_PRIORITY + 3, NULL);

    /* 13. Build the demo UI */
    build_demo_ui();

    /* Main task just idles; LVGL task drives everything */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Heap free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
}
