idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_lcd_ssd1685 lvgl esp_timer
)