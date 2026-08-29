#include "ui_menu_pages.h"
#include "ui_menu_pages_layout.h"

#include <stdint.h>

extern const lv_img_dsc_t ui_menu_title;
extern const lv_img_dsc_t ui_menu_wifi;
extern const lv_img_dsc_t ui_menu_battery;
extern const lv_img_dsc_t ui_menu_arrow_left;
extern const lv_img_dsc_t ui_menu_arrow_right;
extern const lv_img_dsc_t ui_menu_page1;
extern const lv_img_dsc_t ui_menu_page2;
extern const lv_img_dsc_t ui_menu_page3;
extern const lv_img_dsc_t ui_menu_page4;
extern const lv_img_dsc_t ui_menu_page5;
extern const lv_img_dsc_t ui_menu_camera;
extern const lv_font_t ui_font_16;
extern const lv_font_t ui_font_20_menu;

static ui_menu_pages_callbacks_t s_callbacks;

static lv_obj_t *asset(lv_obj_t *parent, const lv_img_dsc_t *source, int x, int y,
                       uint32_t color)
{
    lv_obj_t *img = lv_img_create(parent);
    lv_img_set_src(img, source);
    lv_obj_set_pos(img, x, y);
    lv_obj_set_style_img_recolor(img, lv_color_hex(color), 0);
    lv_obj_set_style_img_recolor_opa(img, LV_OPA_COVER, 0);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_SCROLLABLE);
    return img;
}

static void battery(lv_obj_t *parent, int x, int y, uint8_t percent, bool valid)
{
    const uint8_t level = valid ? (percent > 100 ? 100 : percent) : 0;
    const int width = (16 * level + 99) / 100;
    if (width > 0) {
        lv_obj_t *fill = lv_obj_create(parent);
        lv_obj_remove_style_all(fill);
        lv_obj_set_size(fill, width, 8);
        lv_obj_set_pos(fill, x + 3, y + 4);
        lv_obj_set_style_bg_color(fill, lv_color_hex(level < 10 ? 0xFF3B30 : 0x39E75F), 0);
        lv_obj_set_style_bg_opa(fill, LV_OPA_COVER, 0);
        lv_obj_clear_flag(fill, LV_OBJ_FLAG_SCROLLABLE);
    }
    asset(parent, &ui_menu_battery, x, y, 0xFFFFFF);
}

static void arrow_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && s_callbacks.on_arrow) {
        const int step = (int)(intptr_t)lv_event_get_user_data(event);
        s_callbacks.on_arrow(step, s_callbacks.user_data);
    }
}

static void enter_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && s_callbacks.on_enter) {
        s_callbacks.on_enter(s_callbacks.user_data);
    }
}

static lv_obj_t *hitbox(lv_obj_t *parent, int x, int y, int width, int height,
                        lv_event_cb_t callback, intptr_t user_data)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj, width, height);
    lv_obj_set_pos(obj, x, y);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(obj, callback, LV_EVENT_CLICKED, (void *)user_data);
    return obj;
}

void ui_menu_pages_build(lv_obj_t *parent, uint8_t page_index,
                         bool wifi_visible, uint8_t battery_percent,
                         bool battery_valid,
                         const ui_menu_pages_callbacks_t *callbacks)
{
    if (!parent) return;
    if (page_index >= UI_MENU_PAGE_COUNT) page_index = 0;
    s_callbacks = callbacks ? *callbacks : (ui_menu_pages_callbacks_t){0};

    lv_obj_set_style_bg_color(parent, lv_color_hex(UI_MENU_BACKGROUND_COLOR), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(parent, 40, 0);
    lv_obj_set_style_clip_corner(parent, true, 0);

    asset(parent, &ui_menu_title, ui_menu_title_x[page_index], UI_MENU_TITLE_Y, 0xEFEFEF);
    // WiFi 图标:与电脑连接成功才显示(wifi_visible 由 UI 传 pc_link_connected)
    if (wifi_visible) {
        asset(parent, &ui_menu_wifi, ui_menu_wifi_x[page_index], UI_MENU_WIFI_Y, 0xEFEFEF);
    }
    battery(parent, ui_menu_battery_x[page_index], UI_MENU_BATTERY_Y,
            battery_percent, battery_valid);

    static const lv_img_dsc_t *const main_assets[UI_MENU_PAGE_COUNT] = {
        &ui_menu_page1, &ui_menu_page2, &ui_menu_page3,
        &ui_menu_camera, &ui_menu_page4, &ui_menu_page5,
    };
    static const uint8_t layout_index[UI_MENU_PAGE_COUNT] = {0, 1, 2, 5, 3, 4};
    const uint8_t artwork = layout_index[page_index];
    // 全部主图为纯白 ARGB8565(alpha 承载形状),与相机页同格式,直接显示不 recolor
    lv_obj_t *main_img = lv_img_create(parent);
    lv_img_set_src(main_img, main_assets[page_index]);
    lv_obj_set_pos(main_img, ui_menu_main_x[artwork],
                   ui_menu_main_y[artwork] - (page_index == 3 ? 4 : 0));
    lv_obj_clear_flag(main_img, LV_OBJ_FLAG_SCROLLABLE);

    // 统一功能名称文字:主图资产已裁剪掉底部文字带,这里用同一字体同一
    // 位置渲染,保证 6 个菜单页的字号、间距完全一致(相机页同字号)。
    static const char *const captions[UI_MENU_PAGE_COUNT] = {
        "单点测距", "P2P对测", "平面图", "相机", "测量记录", "设置",
    };
    lv_obj_t *caption = lv_label_create(parent);
    lv_label_set_text(caption, captions[page_index]);
    lv_obj_set_style_text_font(caption, &ui_font_20_menu, 0);
    lv_obj_set_style_text_color(caption, lv_color_hex(0xFFFFFF), 0);
    lv_obj_clear_flag(caption, LV_OBJ_FLAG_SCROLLABLE);
    // 全部页统一固定位置,保证六页文字高度/大小完全一致
    lv_obj_align_to(caption, parent, LV_ALIGN_TOP_MID, 0, 212);

    asset(parent, &ui_menu_arrow_left, UI_MENU_ARROW_LEFT_X, UI_MENU_ARROW_LEFT_Y, 0xACCCE5);
    asset(parent, &ui_menu_arrow_right, UI_MENU_ARROW_RIGHT_X, UI_MENU_ARROW_RIGHT_Y, 0xACCCE5);

    for (int i = 0; ui_menu_dot_visible[page_index] && i < UI_MENU_DOT_COUNT; ++i) {
        lv_obj_t *dot = lv_obj_create(parent);
        lv_obj_set_size(dot, ui_menu_dot_size[i], ui_menu_dot_size[i]);
        lv_obj_set_pos(dot, ui_menu_dot_x[i], ui_menu_dot_y[i]);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(ui_menu_dot_color[page_index][i]), 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    }

    // Touch targets are intentionally larger than the visible artwork.
    hitbox(parent, ui_menu_main_x[artwork], ui_menu_main_y[artwork],
           ui_menu_main_width[artwork], ui_menu_main_height[artwork],
           enter_event, 0);
    hitbox(parent, 0, 102, 40, 58, arrow_event, -1);
    hitbox(parent, UI_MENU_SCREEN_WIDTH - 40, 102, 40, 58, arrow_event, 1);
}
