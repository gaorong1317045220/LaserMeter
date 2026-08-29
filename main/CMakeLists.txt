idf_component_register(
    SRCS "board_self_test.cpp" "nv3030b_lcd.cpp" "nv3030b_boot_brand.c" "device_ui_lvgl.cpp" "fusion_engine.cpp"
         "camera_jpeg_decoder.cpp" "measurement_store.cpp" "app_settings.cpp"
         "single_distance.cpp" "ui_font_16.c" "ui_font_20_menu.c" "ui_menu_assets.c"
         "ui_menu_camera_assets.c" "ui_menu_pages.c" "ui_settings_assets.c" "ui_single_assets.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_adc
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_sdmmc
        esp_driver_spi
        esp_driver_uart
        esp_event
        esp_http_server
        esp_lcd
        esp_netif
        esp_psram
        esp_timer
        esp_wifi
        lvgl__lvgl
        espressif__esp32-camera
        espressif__esp_jpeg
        espressif__esp_new_jpeg
        fatfs
        nvs_flash
        sdmmc
        spi_flash
)
