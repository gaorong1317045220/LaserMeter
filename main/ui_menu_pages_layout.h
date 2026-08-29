// Generated from assets/ui/menu_pages/page1..page6. Do not edit manually.
#pragma once

#define UI_MENU_SCREEN_WIDTH 240
#define UI_MENU_SCREEN_HEIGHT 284
#define UI_MENU_PAGE_COUNT 6
#define UI_MENU_BACKGROUND_COLOR 0x000000u

// Canvas positions in the supplied full-page SVGs; visible bounds are in layout JSON.
// All six pages share the same header positions so the top bar is aligned.
static const int ui_menu_title_x[UI_MENU_PAGE_COUNT] = {52, 52, 52, 52, 52, 52};
#define UI_MENU_TITLE_Y 6
static const int ui_menu_wifi_x[UI_MENU_PAGE_COUNT] = {163, 163, 163, 163, 163, 163};
#define UI_MENU_WIFI_Y 6
static const int ui_menu_battery_x[UI_MENU_PAGE_COUNT] = {193, 193, 193, 193, 193, 193};
#define UI_MENU_BATTERY_Y 10
#define UI_MENU_ARROW_LEFT_X 0
#define UI_MENU_ARROW_LEFT_Y 116
#define UI_MENU_ARROW_RIGHT_X 210
#define UI_MENU_ARROW_RIGHT_Y 116

static const int ui_menu_main_x[UI_MENU_PAGE_COUNT] = {39, 27, 49, 61, 61, 53};
static const int ui_menu_main_y[UI_MENU_PAGE_COUNT] = {75, 111, 60, 63, 74, 63};
static const int ui_menu_main_width[UI_MENU_PAGE_COUNT] = {161, 185, 142, 128, 118, 134};
static const int ui_menu_main_height[UI_MENU_PAGE_COUNT] = {152, 116, 173, 173, 163, 175};

#define UI_MENU_DOT_COUNT 6
static const int ui_menu_dot_visible[UI_MENU_PAGE_COUNT] = {1, 1, 1, 1, 1, 1};
static const int ui_menu_dot_x[UI_MENU_DOT_COUNT] = {54, 78, 102, 126, 150, 174};
static const int ui_menu_dot_y[UI_MENU_DOT_COUNT] = {254, 254, 254, 254, 254, 254};
static const int ui_menu_dot_size[UI_MENU_DOT_COUNT] = {12, 12, 12, 12, 12, 12};
static const unsigned int ui_menu_dot_color[UI_MENU_PAGE_COUNT][UI_MENU_DOT_COUNT] = {
    {0x1677FFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu},
    {0xEFEFEFu, 0x3662ECu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu},
    {0xEFEFEFu, 0xEFEFEFu, 0x3662ECu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu},
    {0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0x3662ECu, 0xEFEFEFu, 0xEFEFEFu},
    {0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0x3662ECu, 0xEFEFEFu},
    {0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0xEFEFEFu, 0x3662ECu},
};
