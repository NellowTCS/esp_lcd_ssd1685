/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: MIT
 *
 * esp_lcd_ssd1685 - ESP-IDF LCD panel driver for Solomon Systech SSD1685
 * (and compatible SSD168x family: SSD1680, SSD1681, SSD1683)
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
 * SSD168x command registers
 * -------------------------------------------------------------------------*/
#define SSD1685_CMD_DRIVER_OUTPUT_CTRL      0x01
#define SSD1685_CMD_GATE_VOLTAGE            0x03
#define SSD1685_CMD_SOURCE_VOLTAGE          0x04
#define SSD1685_CMD_PROG_OTP_INITIAL        0x08
#define SSD1685_CMD_WRITE_REG_INITIAL       0x09
#define SSD1685_CMD_READ_REG_INITIAL        0x0A
#define SSD1685_CMD_BOOST_SOFTSTART         0x0C
#define SSD1685_CMD_GATE_SCAN_START_POS     0x0F
#define SSD1685_CMD_DEEP_SLEEP              0x10
#define SSD1685_CMD_DATA_ENTRY_MODE         0x11
#define SSD1685_CMD_SW_RESET                0x12
#define SSD1685_CMD_HV_READY_DETECT         0x14
#define SSD1685_CMD_VCI_DETECT              0x15
#define SSD1685_CMD_TEMP_SENSOR_CTRL        0x18
#define SSD1685_CMD_TEMP_SENSOR_WRITE       0x1A
#define SSD1685_CMD_TEMP_SENSOR_READ        0x1B
#define SSD1685_CMD_EXT_TEMP_SENSOR_WRITE   0x1C
#define SSD1685_CMD_MASTER_ACTIVATION       0x20
/* FIX (Bug 1): Added Display Update Control 1 - was missing from header.
 * Required to select correct source output range for GDEY029T71H (S8..S175). */
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL1    0x21
#define SSD1685_CMD_DISPLAY_UPDATE_CTRL2    0x22
#define SSD1685_CMD_WRITE_RAM_BW            0x24
#define SSD1685_CMD_WRITE_RAM_RED           0x26
#define SSD1685_CMD_READ_RAM                0x27
#define SSD1685_CMD_VCOM_SENSE              0x28
#define SSD1685_CMD_VCOM_SENSE_DURATION     0x29
#define SSD1685_CMD_PROG_VCOM_OTP           0x2A
#define SSD1685_CMD_WRITE_VCOM_REG          0x2C
#define SSD1685_CMD_READ_OTP_REG            0x2D
#define SSD1685_CMD_USER_ID_READ            0x2E
#define SSD1685_CMD_STATUS_BIT_READ         0x2F
#define SSD1685_CMD_WRITE_LUT               0x32
#define SSD1685_CMD_WRITE_DISP_OPT          0x37
#define SSD1685_CMD_WRITE_USER_ID           0x38
#define SSD1685_CMD_OTP_PROG_MODE           0x39
#define SSD1685_CMD_WRITE_BORDER            0x3C
#define SSD1685_CMD_LUT_OPTION              0x3F
#define SSD1685_CMD_SET_RAM_X_ADDR          0x44
#define SSD1685_CMD_SET_RAM_Y_ADDR          0x45
#define SSD1685_CMD_AUTO_WRITE_RED_PATTERN  0x46
#define SSD1685_CMD_AUTO_WRITE_BW_PATTERN   0x47
#define SSD1685_CMD_SET_RAM_X_COUNTER       0x4E
#define SSD1685_CMD_SET_RAM_Y_COUNTER       0x4F
#define SSD1685_CMD_NOP                     0x7F

/* Data entry mode (cmd 0x11) bit fields */
#define SSD1685_AM_X_FIRST                  0x00
#define SSD1685_AM_Y_FIRST                  0x04
#define SSD1685_ID_X_DECREMENT              0x00
#define SSD1685_ID_X_INCREMENT              0x01
#define SSD1685_ID_Y_DECREMENT              0x00
#define SSD1685_ID_Y_INCREMENT              0x02

/* Display update control 2 (cmd 0x22) sequence flags */
#define SSD1685_DISPLAY_UPDATE_LOAD_LUT     0x80
#define SSD1685_DISPLAY_UPDATE_LOAD_TEMP    0x40
#define SSD1685_DISPLAY_UPDATE_CLOCK_ON     0x80
#define SSD1685_DISPLAY_UPDATE_ANALOGUE_ON  0x40
#define SSD1685_DISPLAY_UPDATE_INITIAL_DISP 0x20
#define SSD1685_DISPLAY_UPDATE_PATTERN      0x10
#define SSD1685_DISPLAY_UPDATE_ANALOGUE_OFF 0x02
#define SSD1685_DISPLAY_UPDATE_CLOCK_OFF    0x01

#define SSD1685_DISP_CTRL2_FULL_REFRESH     0xF7
#define SSD1685_DISP_CTRL2_PARTIAL_REFRESH  0xFF

/* Temperature sensor selection */
#define SSD1685_TEMP_SENSOR_INTERNAL        0x80
#define SSD1685_TEMP_SENSOR_EXTERNAL        0x48

/* Deep sleep modes */
#define SSD1685_DEEP_SLEEP_MODE1            0x01
#define SSD1685_DEEP_SLEEP_MODE2            0x03

/* -------------------------------------------------------------------------
 * Driver configuration structures
 * -------------------------------------------------------------------------*/

typedef enum {
    SSD1685_REFRESH_FULL    = 0,
    SSD1685_REFRESH_PARTIAL = 1,
    SSD1685_REFRESH_FAST    = 2,
} ssd1685_refresh_mode_t;

typedef enum {
    SSD1685_COLOR_PLANE_BW  = 0,
    SSD1685_COLOR_PLANE_RED = 1,
} ssd1685_color_plane_t;

typedef struct {
    gpio_num_t busy_gpio_num;
    uint32_t   busy_timeout_ms;

    uint16_t   panel_width;
    uint16_t   panel_height;

    bool       non_copy_mode;

    ssd1685_refresh_mode_t  default_refresh_mode;

    const uint8_t *custom_lut;
    size_t         custom_lut_size;

    esp_err_t (*on_reset)(void *user_data);
    void *on_reset_user_data;
} esp_lcd_panel_ssd1685_config_t;

/* -------------------------------------------------------------------------
 * Factory function
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_new_panel_ssd1685(
        const esp_lcd_panel_io_handle_t io,
        const esp_lcd_panel_dev_config_t *panel_dev_cfg,
        esp_lcd_panel_handle_t *ret_panel);

/* -------------------------------------------------------------------------
 * Extended EPD-specific operations
 * -------------------------------------------------------------------------*/
esp_err_t esp_lcd_ssd1685_refresh(esp_lcd_panel_handle_t panel,
                                   ssd1685_refresh_mode_t mode);

esp_err_t esp_lcd_ssd1685_set_color_plane(esp_lcd_panel_handle_t panel,
                                           ssd1685_color_plane_t plane);

esp_err_t esp_lcd_ssd1685_clear(esp_lcd_panel_handle_t panel,
                                 uint8_t color_byte);

esp_err_t esp_lcd_ssd1685_sleep(esp_lcd_panel_handle_t panel, uint8_t mode);

esp_err_t esp_lcd_ssd1685_wait_busy(esp_lcd_panel_handle_t panel);

esp_err_t esp_lcd_ssd1685_load_lut(esp_lcd_panel_handle_t panel,
                                    const uint8_t *lut, size_t lut_size);

#ifdef __cplusplus
}
#endif
