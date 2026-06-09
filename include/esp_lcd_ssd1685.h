/*
 * SPDX-FileCopyrightText: 2024 Your Name
 * SPDX-License-Identifier: MIT
 *
 * esp_lcd_ssd1685 - ESP-IDF LCD panel driver for Solomon Systech SSD1685
 * (and compatible SSD168x family: SSD1680, SSD1681, SSD1683)
 *
 * Implements the standard esp_lcd panel interface so it can be used as a
 * drop-in with LVGL or any other stack that calls esp_lcd_panel_draw_bitmap().
 *
 * The SSD1685 is a 4-wire SPI Active Matrix EPD display driver that supports:
 *   - Up to 400 x 300 resolution (panel-dependent)
 *   - Black / White / Red (3-colour) or B/W (2-colour) mode
 *   - Full & partial refresh (panel LUT dependent)
 *   - Internal temperature sensor
 *   - Deep sleep / wake cycle
 *
 * Wiring:
 *   SPI MOSI  → SDA (SDI)
 *   SPI CLK   → SCL
 *   GPIO DC   → D/C#  (data=high, command=low)
 *   GPIO RST  → RES#  (active low)
 *   GPIO BUSY → BSY   (busy = high)
 *   SPI CS    → CS#
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * SSD168x command registers (identical across SSD1680/1681/1683/1685)
 * -------------------------------------------------------------------------*/
#define SSD1685_CMD_DRIVER_OUTPUT_CTRL      0x01  /**< Gate/MUX setting          */
#define SSD1685_CMD_GATE_VOLTAGE            0x03  /**< VGH/VGL levels            */
#define SSD1685_CMD_SOURCE_VOLTAGE          0x04  /**< VSH/VSL levels            */
#define SSD1685_CMD_PROG_OTP_INITIAL        0x08
#define SSD1685_CMD_WRITE_REG_INITIAL       0x09
#define SSD1685_CMD_READ_REG_INITIAL        0x0A
#define SSD1685_CMD_BOOST_SOFTSTART         0x0C  /**< Soft-start phase          */
#define SSD1685_CMD_GATE_SCAN_START_POS     0x0F
#define SSD1685_CMD_DEEP_SLEEP              0x10  /**< Deep sleep mode           */
#define SSD1685_CMD_DATA_ENTRY_MODE         0x11  /**< RAM addressing direction  */
#define SSD1685_CMD_SW_RESET                0x12  /**< Software reset            */
#define SSD1685_CMD_HV_READY_DETECT         0x14
#define SSD1685_CMD_VCI_DETECT              0x15
#define SSD1685_CMD_TEMP_SENSOR_CTRL        0x18  /**< Select int/ext temp sensor*/
#define SSD1685_CMD_TEMP_SENSOR_WRITE       0x1A  /**< Write temperature value   */
#define SSD1685_CMD_TEMP_SENSOR_READ        0x1B  /**< Read temperature value    */
#define SSD1685_CMD_EXT_TEMP_SENSOR_WRITE   0x1C
#define SSD1685_CMD_MASTER_ACTIVATION       0x20  /**< Trigger display update    */
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL1    0x21
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL2    0x22  /**< Sequence flags            */
#define SSD1685_CMD_WRITE_RAM_BW            0x24  /**< Write B/W RAM             */
#define SSD1685_CMD_WRITE_RAM_RED           0x26  /**< Write RED RAM             */
#define SSD1685_CMD_READ_RAM                0x27
#define SSD1685_CMD_VCOM_SENSE              0x28
#define SSD1685_CMD_VCOM_SENSE_DURATION     0x29
#define SSD1685_CMD_PROG_VCOM_OTP           0x2A
#define SSD1685_CMD_WRITE_VCOM_REG          0x2C  /**< VCOM voltage              */
#define SSD1685_CMD_READ_OTP_REG            0x2D
#define SSD1685_CMD_USER_ID_READ            0x2E
#define SSD1685_CMD_STATUS_BIT_READ         0x2F
#define SSD1685_CMD_WRITE_LUT               0x32  /**< Load custom LUT waveform  */
#define SSD1685_CMD_WRITE_DISP_OPT          0x37
#define SSD1685_CMD_WRITE_USER_ID           0x38
#define SSD1685_CMD_OTP_PROG_MODE           0x39
#define SSD1685_CMD_WRITE_BORDER            0x3C  /**< Border waveform control   */
#define SSD1685_CMD_LUT_OPTION              0x3F
#define SSD1685_CMD_SET_RAM_X_ADDR          0x44  /**< RAM X start/end address   */
#define SSD1685_CMD_SET_RAM_Y_ADDR          0x45  /**< RAM Y start/end address   */
#define SSD1685_CMD_AUTO_WRITE_RED_PATTERN  0x46
#define SSD1685_CMD_AUTO_WRITE_BW_PATTERN   0x47
#define SSD1685_CMD_SET_RAM_X_COUNTER      0x4E  /**< RAM X address counter     */
#define SSD1685_CMD_SET_RAM_Y_COUNTER      0x4F  /**< RAM Y address counter     */
#define SSD1685_CMD_NOP                     0x7F

/* Data entry mode (cmd 0x11) bit fields */
#define SSD1685_AM_X_FIRST                  0x00  /**< Address counter moves in X */
#define SSD1685_AM_Y_FIRST                  0x04  /**< Address counter moves in Y */
#define SSD1685_ID_X_DECREMENT              0x00
#define SSD1685_ID_X_INCREMENT              0x01
#define SSD1685_ID_Y_DECREMENT              0x00
#define SSD1685_ID_Y_INCREMENT              0x02

/* Display update control 2 (cmd 0x22) sequence flags */
#define SSD1685_DISPLAY_UPDATE_LOAD_LUT     0x80
#define SSD1685_DISPLAY_UPDATE_LOAD_TEMP    0x40  /* Unused in normal flow       */
#define SSD1685_DISPLAY_UPDATE_CLOCK_ON     0x80
#define SSD1685_DISPLAY_UPDATE_ANALOGUE_ON  0x40
#define SSD1685_DISPLAY_UPDATE_INITIAL_DISP 0x20  /* Used in full refresh        */
#define SSD1685_DISPLAY_UPDATE_PATTERN      0x10
#define SSD1685_DISPLAY_UPDATE_ANALOGUE_OFF 0x02
#define SSD1685_DISPLAY_UPDATE_CLOCK_OFF    0x01

/** Full refresh sequence: enable clk+analog, load initial display, pattern,
 *  then turn off analog+clock */
#define SSD1685_DISP_CTRL2_FULL_REFRESH     0xF7
/** Partial refresh (no initial display stage) */
#define SSD1685_DISP_CTRL2_PARTIAL_REFRESH  0xFF

/* Temperature sensor selection */
#define SSD1685_TEMP_SENSOR_INTERNAL        0x80
#define SSD1685_TEMP_SENSOR_EXTERNAL        0x48

/* Deep sleep modes */
#define SSD1685_DEEP_SLEEP_MODE1            0x01  /**< Retain RAM on wake        */
#define SSD1685_DEEP_SLEEP_MODE2            0x03  /**< Discard RAM on wake       */

/* -------------------------------------------------------------------------
 * Driver configuration structures
 * -------------------------------------------------------------------------*/

/**
 * @brief Refresh mode - controls waveform update sequence
 */
typedef enum {
    SSD1685_REFRESH_FULL    = 0, /**< Full update, slow but ghost-free         */
    SSD1685_REFRESH_PARTIAL = 1, /**< Partial update, fast but may ghost       */
    SSD1685_REFRESH_FAST    = 2, /**< Fast mode if panel OTP supports it       */
} ssd1685_refresh_mode_t;

/**
 * @brief Colour plane written by draw_bitmap
 */
typedef enum {
    SSD1685_COLOR_PLANE_BW  = 0, /**< Write to B/W RAM (cmd 0x24)             */
    SSD1685_COLOR_PLANE_RED = 1, /**< Write to Red RAM (cmd 0x26)             */
} ssd1685_color_plane_t;

/**
 * @brief Device-specific panel configuration.
 *        Pass as vendor_config in esp_lcd_panel_dev_config_t.
 */
typedef struct {
    gpio_num_t busy_gpio_num;        /**< BUSY / BSY pin (active HIGH)         */
    uint32_t   busy_timeout_ms;      /**< Max wait for BUSY low (default 5000) */

    /* Panel physical geometry (in pixels, before any rotation) */
    uint16_t   panel_width;          /**< Source direction pixels              */
    uint16_t   panel_height;         /**< Gate direction pixels                */

    bool       non_copy_mode;        /**< If true, draw_bitmap does NOT trigger
                                          a display refresh automatically.
                                          Caller must call ssd1685_refresh().   */

    ssd1685_refresh_mode_t  default_refresh_mode;  /**< Default refresh mode   */

    /**
     * @brief Optional: point to a 153-byte array containing a full custom LUT
     *        (waveform) to load during init.  NULL = use panel OTP waveform.
     */
    const uint8_t *custom_lut;
    size_t         custom_lut_size;  /**< Bytes; typically 153                 */

    /**
     * @brief Optional: called once after HW reset but before any init commands.
     *        Useful for panels that need a power-sequencing delay.
     */
    esp_err_t (*on_reset)(void *user_data);
    void *on_reset_user_data;
} esp_lcd_panel_ssd1685_config_t;

/* -------------------------------------------------------------------------
 * Factory function
 * -------------------------------------------------------------------------*/

/**
 * @brief Create a new SSD1685 (SSD168x compatible) e-paper panel handle.
 *
 * Usage example:
 * @code
 *   // 1. Init SPI bus (user's responsibility)
 *   spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
 *
 *   // 2. Create panel IO
 *   esp_lcd_panel_io_spi_config_t io_cfg = {
 *       .cs_gpio_num        = PIN_CS,
 *       .dc_gpio_num        = PIN_DC,
 *       .spi_mode           = 0,
 *       .pclk_hz            = 4 * 1000 * 1000,
 *       .trans_queue_depth  = 4,
 *       .lcd_cmd_bits       = 8,
 *       .lcd_param_bits     = 8,
 *   };
 *   esp_lcd_panel_io_handle_t io_handle;
 *   esp_lcd_new_panel_io_spi(SPI2_HOST, &io_cfg, &io_handle);
 *
 *   // 3. Configure SSD1685 specific options
 *   esp_lcd_panel_ssd1685_config_t epd_cfg = {
 *       .busy_gpio_num  = PIN_BUSY,
 *       .panel_width    = 296,
 *       .panel_height   = 128,
 *   };
 *
 *   // 4. Create panel
 *   esp_lcd_panel_dev_config_t panel_cfg = {
 *       .reset_gpio_num = PIN_RST,
 *       .bits_per_pixel = 1,
 *       .vendor_config  = &epd_cfg,
 *   };
 *   esp_lcd_panel_handle_t panel;
 *   esp_lcd_new_panel_ssd1685(io_handle, &panel_cfg, &panel);
 *
 *   // 5. Standard init sequence
 *   esp_lcd_panel_reset(panel);
 *   esp_lcd_panel_init(panel);
 *
 *   // 6. Draw something (1 bit per pixel, 0 = black, 1 = white)
 *   esp_lcd_panel_draw_bitmap(panel, 0, 0, 296, 128, bitmap);
 * @endcode
 *
 * @param io             Panel IO handle (SPI)
 * @param panel_dev_cfg  Generic panel device config; set vendor_config to
 *                       an esp_lcd_panel_ssd1685_config_t pointer.
 * @param ret_panel      Returned panel handle
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_new_panel_ssd1685(
        const esp_lcd_panel_io_handle_t io,
        const esp_lcd_panel_dev_config_t *panel_dev_cfg,
        esp_lcd_panel_handle_t *ret_panel);

/* -------------------------------------------------------------------------
 * Extended (EPD-specific) operations beyond the standard panel interface
 * -------------------------------------------------------------------------*/

/**
 * @brief Explicitly trigger a display refresh (show updated RAM on screen).
 *
 * Call this after draw_bitmap() if non_copy_mode is true, or whenever you
 * want to commit queued writes without drawing new data.
 *
 * @param panel   Panel handle
 * @param mode    Refresh mode (full / partial / fast)
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_ssd1685_refresh(esp_lcd_panel_handle_t panel,
                                   ssd1685_refresh_mode_t mode);

/**
 * @brief Select which colour plane subsequent draw_bitmap calls write to.
 *
 * For 2-colour (B/W) displays: only SSD1685_COLOR_PLANE_BW is relevant.
 * For 3-colour (B/W/R) displays: call draw_bitmap twice - once per plane.
 *
 * @param panel  Panel handle
 * @param plane  Colour plane to target
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_ssd1685_set_color_plane(esp_lcd_panel_handle_t panel,
                                           ssd1685_color_plane_t plane);

/**
 * @brief Load a full-screen solid colour into both RAM planes, then refresh.
 *
 * Useful for a clean first-boot clear.
 *
 * @param panel      Panel handle
 * @param color_byte 0xFF = all white, 0x00 = all black
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_ssd1685_clear(esp_lcd_panel_handle_t panel,
                                 uint8_t color_byte);

/**
 * @brief Enter hardware deep sleep (command 0x10).
 *
 * The panel must be re-initialised (esp_lcd_panel_reset + esp_lcd_panel_init)
 * before it can be used again.
 *
 * @param panel Panel handle
 * @param mode  SSD1685_DEEP_SLEEP_MODE1 (RAM retained) or MODE2 (RAM lost)
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_ssd1685_sleep(esp_lcd_panel_handle_t panel, uint8_t mode);

/**
 * @brief Block until the BUSY pin goes low (panel ready) or timeout elapses.
 *
 * @param panel Panel handle
 * @return ESP_OK if ready, ESP_ERR_TIMEOUT if timed out
 */
esp_err_t esp_lcd_ssd1685_wait_busy(esp_lcd_panel_handle_t panel);

/**
 * @brief Load a custom LUT waveform into the panel.
 *
 * The SSD168x LUT register (0x32) accepts up to 153 bytes.
 * You may also optionally override voltage registers 0x03, 0x04 and 0x2C.
 *
 * @param panel    Panel handle
 * @param lut      Pointer to waveform bytes
 * @param lut_size Number of bytes (typically 153)
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_ssd1685_load_lut(esp_lcd_panel_handle_t panel,
                                    const uint8_t *lut, size_t lut_size);

#ifdef __cplusplus
}
#endif
