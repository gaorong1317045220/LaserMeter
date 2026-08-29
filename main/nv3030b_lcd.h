#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

esp_err_t nv3030b_lcd_init();
esp_err_t nv3030b_lcd_fill(uint16_t rgb565);
esp_err_t nv3030b_lcd_show_test_pattern();
// Boot brand screen: dark background with a brand band and an accent line,
// shown while the firmware initialises (before the LVGL UI starts).
esp_err_t nv3030b_lcd_show_boot_brand();
// Pixels must be RGB565 in LCD wire byte order. The driver copies PSRAM data
// through its internal DMA stripe buffer before sending it to the panel.
esp_err_t nv3030b_lcd_present(const uint16_t *pixels, size_t pixel_count);
esp_err_t nv3030b_lcd_present_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                   const uint16_t *pixels, size_t pixel_count);
