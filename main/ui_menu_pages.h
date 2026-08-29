#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void (*on_arrow)(int step, void *user_data);
    void (*on_enter)(void *user_data);
    void *user_data;
} ui_menu_pages_callbacks_t;

// Builds one of the six menu pages at the exact 240x284 design coordinates.
void ui_menu_pages_build(lv_obj_t *parent, uint8_t page_index,
                         bool wifi_visible, uint8_t battery_percent,
                         bool battery_valid,
                         const ui_menu_pages_callbacks_t *callbacks);

#ifdef __cplusplus
}
#endif
