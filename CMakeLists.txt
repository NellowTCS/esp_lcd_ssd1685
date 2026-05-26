idf_component_register(
    SRCS "esp_lcd_ssd1685.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_lcd
    PRIV_REQUIRES esp_timer
)