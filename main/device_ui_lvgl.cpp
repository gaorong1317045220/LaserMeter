// LVGL 8.4 UI for the laser meter.
// Camera Task -> RGB565 double buffers -> LVGL transparent overlay -> LCD DMA.

#include "device_ui.h"
#include "app_settings.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

#include "camera_jpeg_decoder.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "laser_camera_calibration.h"
#include "nv3030b_lcd.h"
#include "ui_menu_pages.h"

extern "C" const lv_font_t ui_font_16;
extern "C" const lv_img_dsc_t ui_menu_wifi;
extern "C" const lv_img_dsc_t ui_menu_battery;
extern "C" const lv_img_dsc_t ui_settings_back;
extern "C" const lv_img_dsc_t ui_settings_title;
extern "C" const lv_img_dsc_t ui_settings_card;
extern "C" const lv_img_dsc_t ui_single_title;
extern "C" const lv_img_dsc_t ui_single_ref_front;
extern "C" const lv_img_dsc_t ui_single_ref_rear;
extern "C" const lv_img_dsc_t ui_single_ref_tripod;
extern "C" const lv_img_dsc_t ui_single_zoom_1x;
extern "C" const lv_img_dsc_t ui_single_zoom_2x;
extern "C" const lv_img_dsc_t ui_single_zoom_5x;
extern "C" const lv_img_dsc_t ui_single_laser_label;
extern "C" const lv_img_dsc_t ui_single_laser_off;
extern "C" const lv_img_dsc_t ui_single_laser_on;
extern "C" const lv_img_dsc_t ui_single_level_track;
extern "C" const lv_img_dsc_t ui_single_crosshair;
extern "C" const lv_img_dsc_t ui_single_bottom_history;
extern "C" const lv_img_dsc_t ui_single_bottom_measure;
extern "C" const lv_img_dsc_t ui_single_bottom_back;
extern "C" const lv_img_dsc_t ui_single_bottom_save;
extern "C" const lv_img_dsc_t ui_single_bottom_delete;
extern "C" const lv_img_dsc_t ui_single_bottom_select;
extern "C" const lv_img_dsc_t ui_single_bottom_export;

namespace {

constexpr int kWidth = 240;
constexpr int kHeight = 284;
constexpr int kCameraCropRows = 18;
constexpr int kDrawRows = 40;
constexpr uint32_t kBg = 0x0D1114;
constexpr uint32_t kPanel = 0x242A2E;
constexpr uint32_t kWhite = 0xFFFFFF;
constexpr uint32_t kMuted = 0xA6A6A6;
constexpr uint32_t kGreen = 0x39E75F;
constexpr uint32_t kRed = 0xFF5548;
constexpr uint32_t kAmber = 0xFFC400;
constexpr uint8_t kHistorySlotCount = 5;

enum class Page : uint8_t {
    MENU, SINGLE, STORAGE, RECORD_DETAIL, RECORD_DELETE, SETTINGS, WEB,
    SENSORS, MEASURE, ROOM, IMU_CAL, CAMERA
};

const char *TAG = "device_ui_lvgl";
DeviceUiCallbacks s_cb = {};
TaskHandle_t s_lvgl_task = nullptr;
TaskHandle_t s_camera_task = nullptr;
DeviceUiState *s_ui_state = nullptr;
std::atomic<bool> s_camera_requested{false};
portMUX_TYPE s_camera_mux = portMUX_INITIALIZER_UNLOCKED;
uint16_t *s_camera[2] = {nullptr, nullptr};
uint16_t *s_record_decode = nullptr;
int s_camera_front = -1;
int s_camera_reading = -1;
uint32_t s_camera_sequence = 0;

lv_color_t *s_draw_a = nullptr;
lv_color_t *s_draw_b = nullptr;
lv_color_t *s_camera_image = nullptr;
lv_color_t *s_thumbnail_pixels = nullptr;
lv_img_dsc_t s_thumbnail_dsc[DEVICE_UI_RECORD_CAPACITY] = {};
uint32_t s_thumbnail_record_id[DEVICE_UI_RECORD_CAPACITY] = {};
lv_img_dsc_t s_camera_dsc = {};
lv_disp_draw_buf_t s_draw_buf = {};
lv_disp_drv_t s_disp_drv = {};
lv_indev_drv_t s_indev_drv = {};
lv_obj_t *s_root = nullptr;
lv_obj_t *s_camera_view = nullptr;
lv_obj_t *s_camera_capture_status = nullptr;
lv_obj_t *s_value = nullptr;
lv_obj_t *s_status = nullptr;
lv_obj_t *s_error = nullptr;
lv_obj_t *s_pitch_text = nullptr;
lv_obj_t *s_level_marker = nullptr;
lv_obj_t *s_dynamic = nullptr;
lv_obj_t *s_zoom_text = nullptr;
lv_obj_t *s_reference_text = nullptr;
lv_obj_t *s_laser_chip = nullptr;
lv_obj_t *s_laser_text = nullptr;
lv_obj_t *s_save_button = nullptr;
lv_obj_t *s_reference_image = nullptr;
lv_obj_t *s_zoom_image = nullptr;
lv_obj_t *s_laser_image = nullptr;
lv_obj_t *s_crosshair_image = nullptr;
lv_obj_t *s_center_action_image = nullptr;
lv_obj_t *s_result_dot = nullptr;
lv_obj_t *s_value_panel = nullptr;
lv_obj_t *s_battery_fill = nullptr;
Page s_page = Page::MENU;
Page s_storage_return = Page::SINGLE;
Page s_hardware_page = static_cast<Page>(255);
bool s_rebuild = true;
std::atomic<int> s_external_page_request{-1};
bool s_rendered_startup = false;
uint32_t s_startup_shown_tick = 0;  // 自检页首次渲染时刻(至少展示 2 秒)
uint32_t s_last_camera_sequence = 0;
uint32_t s_last_measure = 0, s_last_back = 0, s_last_ok = 0;
uint32_t s_last_long_measure = 0, s_last_long_back = 0, s_last_long_ok = 0;
uint8_t s_menu_index = 0;
uint8_t s_record_selected = 0;
uint8_t s_history_first = 0;       // 按键导航时列表首个可视记录(高亮卡固定在视口内)
bool s_history_key_mode = false;   // true=refresh 用 s_history_first;false=按滚动位置推导
uint32_t s_program_scroll_tick = 0; // 编程滚动时间戳,用于吞掉其触发的事件(防误清 key_mode)
uint8_t s_imu_cal_step = 0;
uint8_t s_zoom = 0;
uint8_t s_settings_index = 0;
bool s_auto_save_single_pending = false; // 长按测量=测量并自动保存
uint32_t s_pending_delete_id = 0;
// 简易确认框状态:非零时显示"测量=确认 Back=取消"
uint8_t s_confirm = 0;
lv_obj_t *s_confirm_bar = nullptr;
uint32_t s_record_selection_mask = 0;
// 缩略图全局缓存(按记录 id):滚动换槽位时避免重复读 SD 解码
constexpr uint8_t kThumbCacheCount = 8;
struct ThumbCacheEntry {
    uint32_t id = 0;
    bool valid = false;
    lv_color_t pixels[50 * 60];
};
static ThumbCacheEntry *s_thumb_cache = nullptr;
static uint8_t s_thumb_cache_next = 0;
bool s_rendered_laser_on = false;
bool s_rendered_result_valid = false;
uint8_t s_rendered_unit = 255;
lv_obj_t *s_history_list = nullptr;
lv_obj_t *s_history_card[kHistorySlotCount] = {};
lv_obj_t *s_history_thumb[kHistorySlotCount] = {};
lv_obj_t *s_history_type[kHistorySlotCount] = {};
lv_obj_t *s_history_value[kHistorySlotCount] = {};
lv_obj_t *s_history_date[kHistorySlotCount] = {};
lv_obj_t *s_history_time[kHistorySlotCount] = {};
lv_obj_t *s_history_box[kHistorySlotCount] = {};
lv_obj_t *s_history_check[kHistorySlotCount] = {};
uint8_t s_history_record_index[kHistorySlotCount] = {255, 255, 255, 255, 255};
bool s_thumbnail_loaded[DEVICE_UI_RECORD_CAPACITY] = {};

uint16_t swap16(uint16_t v) { return static_cast<uint16_t>((v << 8) | (v >> 8)); }

// The JPEG calibration is in the original landscape camera coordinates. The
// live preview is VGA, then the decoder rotates it to portrait; the UI mirrors
// it horizontally and removes 18 rows at each end. Keep this transform here so
// the calibrated laser reticle follows exactly the same pixel path as video.
constexpr float kPreviewWidthPx = 640.0f;
constexpr float kPreviewHeightPx = 480.0f;
constexpr float kDecodedLandscapeWidthPx = 320.0f;
constexpr float kDecodedPortraitWidthPx = 240.0f;
constexpr float kDecodedPortraitHeightPx = kDecodedLandscapeWidthPx;
// copy_camera_crop_to_lvgl() applies one more 120x142 centre crop and scales
// it to the 240x284 LCD area in 2x mode. Keep the projection in that exact
// coordinate system so the calibrated reticle and live image share a pixel.
constexpr float kZoomDisplayCropWidthPx = 120.0f;
constexpr float kZoomDisplayCropHeightPx = 142.0f;
constexpr float kZoomDisplayCropX = (kDecodedPortraitWidthPx - kZoomDisplayCropWidthPx) * 0.5f;
constexpr float kZoomDisplayCropY = (kDecodedPortraitHeightPx - kZoomDisplayCropHeightPx) * 0.5f;
constexpr float kSingleRearOffsetMm = 126.5f;
constexpr float kSingleFrontOffsetMm = -2.0f;
constexpr float kSingleTripodOffsetMm = 18.0f;

bool project_laser_to_display(float range_m, bool zoom_2x,
                              float *display_x, float *display_y)
{
#if LASER_CAMERA_CALIBRATION_VALID
    if (!display_x || !display_y || !std::isfinite(range_m) || range_m <= 0.0f) return false;
    const float fx = LASER_CAMERA_CALIBRATION_FX_PX *
                     (kPreviewWidthPx / LASER_CAMERA_CALIBRATION_IMAGE_WIDTH_PX);
    const float fy = LASER_CAMERA_CALIBRATION_FY_PX *
                     (kPreviewHeightPx / LASER_CAMERA_CALIBRATION_IMAGE_HEIGHT_PX);
    const float cx = LASER_CAMERA_CALIBRATION_CX_PX *
                     (kPreviewWidthPx / LASER_CAMERA_CALIBRATION_IMAGE_WIDTH_PX);
    const float cy = LASER_CAMERA_CALIBRATION_CY_PX *
                     (kPreviewHeightPx / LASER_CAMERA_CALIBRATION_IMAGE_HEIGHT_PX);

    const float cam_x = LASER_CAMERA_P0_CAM_X + LASER_CAMERA_DIR_CAM_X * range_m;
    const float cam_y = LASER_CAMERA_P0_CAM_Y + LASER_CAMERA_DIR_CAM_Y * range_m;
    const float cam_z = LASER_CAMERA_P0_CAM_Z + LASER_CAMERA_DIR_CAM_Z * range_m;
    if (cam_z <= 1e-5f) return false;
    const float xn = cam_x / cam_z;
    const float yn = cam_y / cam_z;
    const float radius2 = xn * xn + yn * yn;
    const float radial = 1.0f + LASER_CAMERA_CALIBRATION_DIST_K1 * radius2 +
                         LASER_CAMERA_CALIBRATION_DIST_K2 * radius2 * radius2 +
                         LASER_CAMERA_CALIBRATION_DIST_K3 * radius2 * radius2 * radius2;
    const float xd = xn * radial +
                     2.0f * LASER_CAMERA_CALIBRATION_DIST_P1 * xn * yn +
                     LASER_CAMERA_CALIBRATION_DIST_P2 * (radius2 + 2.0f * xn * xn);
    const float yd = yn * radial +
                     LASER_CAMERA_CALIBRATION_DIST_P1 * (radius2 + 2.0f * yn * yn) +
                     2.0f * LASER_CAMERA_CALIBRATION_DIST_P2 * xn * yn;
    const float u = fx * xd + cx;
    const float v = fy * yd + cy;

    if (zoom_2x) {
        // 2x uses the same 320x240 decoder output as 1x, then centre-crops
        // 120x142 and scales to 240x284 while mirroring horizontally. In the
        // rotated decoder frame x=v/2 and y=319-u/2.
        const float frame_x = v * (kDecodedLandscapeWidthPx / kPreviewWidthPx);
        const float frame_y = (kDecodedLandscapeWidthPx - 1.0f) -
                              u * (kDecodedLandscapeWidthPx / kPreviewWidthPx);
        *display_x = (kZoomDisplayCropX + kZoomDisplayCropWidthPx - 1.0f - frame_x) *
                     (kWidth / kZoomDisplayCropWidthPx);
        *display_y = (frame_y - kZoomDisplayCropY) *
                     (kHeight / kZoomDisplayCropHeightPx);
    } else {
        // 1x first scales VGA to 320x240 before rotating and mirroring.
        *display_x = 239.0f - v * 0.5f;
        *display_y = 301.0f - u * 0.5f;
    }
    return std::isfinite(*display_x) && std::isfinite(*display_y);
#else
    (void)range_m;
    (void)zoom_2x;
    (void)display_x;
    (void)display_y;
    return false;
#endif
}

float single_raw_range_m(const DeviceUiState &state)
{
    if (!state.single_result_valid || state.single_result.distance_mm <= 0) return 1.0f;
    float offset_mm = kSingleRearOffsetMm;
    switch (state.single_result.reference) {
    case DistanceReference::FRONT: offset_mm = kSingleFrontOffsetMm; break;
    case DistanceReference::TRIPOD: offset_mm = kSingleTripodOffsetMm; break;
    default: break;
    }
    const float raw_mm = static_cast<float>(state.single_result.distance_mm) - offset_mm;
    return raw_mm > 1.0f ? raw_mm / 1000.0f : 1.0f;
}

void update_calibrated_crosshair(float range_m)
{
    if (!s_crosshair_image) return;
    float x = 0.0f, y = 0.0f;
    if (!project_laser_to_display(range_m, s_zoom != 0, &x, &y)) return;
    const int image_width = static_cast<int>(ui_single_crosshair.header.w);
    const int image_height = static_cast<int>(ui_single_crosshair.header.h);
    const int left = std::clamp(static_cast<int>(std::lround(x - image_width * 0.5f)),
                                0, std::max(0, kWidth - image_width));
    const int top = std::clamp(static_cast<int>(std::lround(y - image_height * 0.5f)),
                               0, std::max(0, kHeight - image_height));
    if (lv_obj_get_x(s_crosshair_image) != left || lv_obj_get_y(s_crosshair_image) != top) {
        lv_obj_set_pos(s_crosshair_image, left, top);
    }
}

void update_calibrated_crosshair(const DeviceUiState &state)
{
    if (s_page == Page::SINGLE) {
        update_calibrated_crosshair(single_raw_range_m(state));
    } else if (s_page == Page::MEASURE) {
        const float range_m = state.laser_mm > 0 ? state.laser_mm / 1000.0f : 1.0f;
        update_calibrated_crosshair(range_m);
    }
}

// The product preview behaves like a viewfinder: after rotating the landscape
// sensor frame into portrait, mirror it horizontally for the operator.  The
// original JPEG saved on SD remains untouched.
void copy_camera_crop_to_lvgl(const uint16_t *frame, uint8_t zoom = 0)
{
    if (!frame || !s_camera_image) return;
    // UI stores zoom as an index: 0 = 1x, 1 = 2x.
    if (zoom != 0) {
        constexpr int crop_width = 120;
        constexpr int crop_height = 142;
        constexpr int crop_x = (kWidth - crop_width) / 2;
        constexpr int crop_y = (CAMERA_JPEG_ROTATED_HEIGHT - crop_height) / 2;
        for (int y = 0; y < kHeight; ++y) {
            const int source_y = crop_y + y * crop_height / kHeight;
            for (int x = 0; x < kWidth; ++x) {
                const int source_x = crop_x + crop_width - 1 - x * crop_width / kWidth;
                s_camera_image[static_cast<size_t>(y) * kWidth + x].full =
                    swap16(frame[static_cast<size_t>(source_y) * kWidth + source_x]);
            }
        }
        return;
    }
    const uint16_t *crop = frame + kCameraCropRows * kWidth;
    for (int y = 0; y < kHeight; ++y) {
        const size_t row = static_cast<size_t>(y) * kWidth;
        for (int x = 0; x < kWidth; ++x) {
            s_camera_image[row + x].full = swap16(crop[row + (kWidth - 1 - x)]);
        }
    }
}

void style_screen(lv_obj_t *obj, uint32_t color = kBg)
{
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
}

lv_obj_t *text(lv_obj_t *parent, const char *value, int x, int y,
               uint32_t color = kWhite, const lv_font_t *font = &ui_font_16)
{
    lv_obj_t *obj = lv_label_create(parent);
    lv_label_set_text(obj, value);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_style_text_color(obj, lv_color_hex(color), 0);
    lv_obj_set_style_text_font(obj, font, 0);
    return obj;
}

lv_obj_t *image_asset(lv_obj_t *parent, const lv_img_dsc_t *source, int x, int y)
{
    lv_obj_t *image = lv_img_create(parent);
    lv_img_set_src(image, source);
    lv_obj_set_pos(image, x, y);
    lv_obj_clear_flag(image, LV_OBJ_FLAG_SCROLLABLE);
    return image;
}

lv_obj_t *transparent_hitbox(lv_obj_t *parent, int x, int y, int width, int height,
                             lv_event_cb_t callback, intptr_t data)
{
    lv_obj_t *hit = lv_obj_create(parent);
    lv_obj_remove_style_all(hit);
    lv_obj_set_size(hit, width, height);
    lv_obj_set_pos(hit, x, y);
    lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(hit, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(hit, callback, LV_EVENT_CLICKED, reinterpret_cast<void *>(data));
    return hit;
}

lv_obj_t *overlay_bar(lv_obj_t *parent, int y, int height)
{
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_set_size(bar, kWidth, height);
    lv_obj_set_pos(bar, 0, y);
    style_screen(bar, 0x000000);
    lv_obj_set_style_bg_opa(bar, 191, 0);
    return bar;
}

void add_battery(lv_obj_t *parent, int x, int y, const DeviceUiState &state)
{
    const uint8_t level = state.battery_valid ? std::min<uint8_t>(state.battery_percent, 100) : 0;
    const int width = (16 * level + 99) / 100;
    s_battery_fill = lv_obj_create(parent);
    lv_obj_remove_style_all(s_battery_fill);
    lv_obj_set_size(s_battery_fill, std::max(1, width), 8);
    lv_obj_set_pos(s_battery_fill, x + 3, y + 4);
    lv_obj_set_style_bg_color(s_battery_fill,
                              lv_color_hex(level < 10 ? 0xFF3B30 : kGreen), 0);
    lv_obj_set_style_bg_opa(s_battery_fill, width ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(s_battery_fill, LV_OBJ_FLAG_SCROLLABLE);
    image_asset(parent, &ui_menu_battery, x, y);
}

void update_battery(const DeviceUiState &state)
{
    if (!s_battery_fill) return;
    const uint8_t level = state.battery_valid ? std::min<uint8_t>(state.battery_percent, 100) : 0;
    const int width = (16 * level + 99) / 100;
    lv_obj_set_width(s_battery_fill, std::max(1, width));
    lv_obj_set_style_bg_opa(s_battery_fill, width ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(s_battery_fill,
                              lv_color_hex(level < 10 ? 0xFF3B30 : kGreen), 0);
}

void style_button(lv_obj_t *obj, uint32_t border = kMuted, bool filled = true)
{
    lv_obj_set_style_bg_color(obj, lv_color_hex(kPanel), 0);
    lv_obj_set_style_bg_opa(obj, filled ? LV_OPA_80 : LV_OPA_30, 0);
    lv_obj_set_style_border_color(obj, lv_color_hex(border), 0);
    lv_obj_set_style_border_width(obj, 2, 0);
    lv_obj_set_style_radius(obj, 13, 0);
    lv_obj_set_style_shadow_width(obj, 0, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x4A5054), LV_STATE_PRESSED);
}

lv_obj_t *button(lv_obj_t *parent, const char *caption, int x, int y, int w, int h,
                 lv_event_cb_t callback, intptr_t data = 0, uint32_t border = kMuted,
                 bool filled = true)
{
    lv_obj_t *obj = lv_btn_create(parent);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_pos(obj, x, y);
    style_button(obj, border, filled);
    if (callback) lv_obj_add_event_cb(obj, callback, LV_EVENT_CLICKED,
                                      reinterpret_cast<void *>(data));
    lv_obj_t *label = text(obj, caption, 0, 0);
    lv_obj_center(label);
    return obj;
}

void go(Page page)
{
    if (page != s_page) {
        ESP_LOGI(TAG, "page transition %u -> %u", static_cast<unsigned>(s_page),
                 static_cast<unsigned>(page));
    }
    s_page = page;
    s_rebuild = true;
    s_confirm = 0;  // 页面切换时清除确认框
    if (s_lvgl_task) xTaskNotifyGive(s_lvgl_task);
}

void event_go(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        go(static_cast<Page>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event))));
    }
}

void clear_screen()
{
    lv_obj_clean(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(kBg), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    s_root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_root, kWidth, kHeight);
    lv_obj_set_pos(s_root, 0, 0);
    style_screen(s_root);
    // 与主菜单一致的圆角屏效果(全 UI 统一)
    lv_obj_set_style_radius(s_root, 40, 0);
    lv_obj_set_style_clip_corner(s_root, true, 0);
    s_camera_view = s_camera_capture_status = s_value = s_status = s_error = s_pitch_text = nullptr;
    s_level_marker = s_dynamic = s_zoom_text = nullptr;
    s_reference_text = s_laser_chip = s_laser_text = s_save_button = nullptr;
    s_reference_image = s_zoom_image = s_laser_image = s_crosshair_image = nullptr;
    s_center_action_image = nullptr;
    s_result_dot = s_value_panel = nullptr;
    s_battery_fill = nullptr;
    s_history_list = nullptr;
    s_confirm_bar = nullptr;  // 屏幕清理后确认条对象失效,防止悬垂指针崩溃
    for (uint8_t slot = 0; slot < kHistorySlotCount; ++slot) {
        s_history_card[slot] = s_history_thumb[slot] = nullptr;
        s_history_type[slot] = s_history_value[slot] = nullptr;
        s_history_date[slot] = s_history_time[slot] = nullptr;
        s_history_box[slot] = s_history_check[slot] = nullptr;
        s_history_record_index[slot] = 255;
    }
    lv_obj_invalidate(lv_scr_act());
}

void add_header(const DeviceUiState &state, const char *title, Page back)
{
    // 与主菜单统一:黑色顶栏 + 返回箭头资产 + 标题 + wifi/电池资产
    lv_obj_t *bar = lv_obj_create(s_root);
    lv_obj_set_size(bar, kWidth, 36);
    lv_obj_set_pos(bar, 0, 0);
    style_screen(bar, 0x000000);
    lv_obj_set_style_bg_opa(bar, 191, 0);
    if (back != static_cast<Page>(255)) {
        lv_obj_t *back_img = image_asset(bar, &ui_settings_back, 6, 4);
        lv_obj_set_style_img_recolor(back_img, lv_color_hex(0xACCCE5), 0);
        lv_obj_set_style_img_recolor_opa(back_img, LV_OPA_COVER, 0);
        transparent_hitbox(bar, 0, 0, 44, 36, event_go, static_cast<intptr_t>(back));
    }
    text(bar, title, back == static_cast<Page>(255) ? 16 : 44, 8);
    // WiFi 图标按实际连接状态显示:与电脑连接成功后才显示
    if (state.pc_link_connected) {
        lv_obj_t *wifi = image_asset(bar, &ui_menu_wifi, 163, 6);
        lv_obj_set_style_img_recolor(wifi, lv_color_hex(0xEFEFEF), 0);
        lv_obj_set_style_img_recolor_opa(wifi, LV_OPA_COVER, 0);
    }
    add_battery(bar, 193, 10, state);
}

void build_startup(const DeviceUiState &state)
{
    clear_screen();
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    text(s_root, "激光测距仪", 72, 24, kWhite);
    text(s_root, "设备自检", 84, 67, kWhite);
    const DeviceCheckState checks[] = {state.check_lcd, state.check_i2c, state.check_input,
        state.check_sd, state.check_camera, state.check_laser_uart};
    const char *names[] = {"显示", "总线", "按键", "存储卡", "摄像头", "激光器"};
    for (int i = 0; i < 6; ++i) {
        const char *mark = checks[i] == DeviceCheckState::PASS ? "正常" :
                           checks[i] == DeviceCheckState::FAIL ? "异常" : "...";
        const uint32_t color = checks[i] == DeviceCheckState::PASS ? kGreen :
                               checks[i] == DeviceCheckState::FAIL ? kRed : kMuted;
        text(s_root, names[i], 42, 104 + i * 24, kMuted);
        text(s_root, mark, 155, 104 + i * 24, color);
    }
}

void designed_menu_arrow(int step, void *)
{
    s_menu_index = static_cast<uint8_t>((s_menu_index + 6 + step) % 6);
    s_rebuild = true;
}

void designed_menu_enter(void *)
{
    static const Page targets[] = {Page::SINGLE, Page::MEASURE, Page::ROOM,
                                   Page::CAMERA, Page::STORAGE, Page::SETTINGS};
    if (s_menu_index < sizeof(targets) / sizeof(targets[0])) {
        if (targets[s_menu_index] == Page::STORAGE) s_storage_return = Page::MENU;
        go(targets[s_menu_index]);
    }
}

void build_menu(const DeviceUiState &state)
{
    clear_screen();
    const ui_menu_pages_callbacks_t callbacks = {
        .on_arrow = designed_menu_arrow,
        .on_enter = designed_menu_enter,
        .user_data = nullptr,
    };
    // WiFi 图标按实际连接状态显示(与电脑连接成功才显示),与其它页面一致
    ui_menu_pages_build(s_root, s_menu_index, state.pc_link_connected,
                        state.battery_percent, state.battery_valid, &callbacks);
}

void event_single(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    switch (reinterpret_cast<intptr_t>(lv_event_get_user_data(event))) {
    case 0: if (s_cb.single_cycle_reference) s_cb.single_cycle_reference(); break;
    case 1:
        if (s_ui_state && s_ui_state->single_result_valid) {
            if (s_cb.single_save) s_cb.single_save();
        } else if (s_cb.single_trigger) s_cb.single_trigger();
        break;
    case 2: s_storage_return = Page::SINGLE; go(Page::STORAGE); break;
    case 3:
        s_zoom = static_cast<uint8_t>((s_zoom + 1) % 2);  // 1x/2x
        if (s_cb.set_camera_zoom) s_cb.set_camera_zoom(s_zoom ? 1 : 0);
        if (s_zoom_image) {
            static const lv_img_dsc_t *const zoom_assets[] = {
                &ui_single_zoom_1x, &ui_single_zoom_2x,
            };
            lv_img_set_src(s_zoom_image, zoom_assets[s_zoom]);
        }
        break;
    case 4:
        if (s_ui_state && s_ui_state->single_state == SingleDistanceState::AIMING) {
            if (s_cb.single_cancel) s_cb.single_cancel();
        } else if (s_cb.single_trigger) {
            s_cb.single_trigger();
        }
        break;
    case 5: go(Page::MENU); break;
    }
}

void build_single(const DeviceUiState &state)
{
    clear_screen();
    s_camera_requested.store(true, std::memory_order_release);
    s_camera_view = lv_img_create(s_root);
    lv_img_set_src(s_camera_view, &s_camera_dsc);
    lv_obj_set_size(s_camera_view, kWidth, kHeight);
    lv_obj_set_pos(s_camera_view, 0, 0);

    // 统一顶栏(与其它页面同款:黑色信息栏 + 标题 + WiFi/电池资产)
    add_header(state, "单点测距", static_cast<Page>(255));

    static const lv_img_dsc_t *const reference_assets[] = {
        &ui_single_ref_rear, &ui_single_ref_front, &ui_single_ref_tripod,
    };
    const bool rear_reference = state.single_selected_reference == DistanceReference::REAR;
    s_reference_image = image_asset(s_root,
        reference_assets[static_cast<uint8_t>(state.single_selected_reference)],
        rear_reference ? 9 : 8, rear_reference ? 45 : 43);
    static const lv_img_dsc_t *const zoom_assets[] = {
        &ui_single_zoom_1x, &ui_single_zoom_2x,
    };
    s_zoom_image = image_asset(s_root, zoom_assets[s_zoom % 2], 108, 44);
    const bool laser_on = state.single_state == SingleDistanceState::AIMING ||
                          state.single_state == SingleDistanceState::MEASURING;
    s_rendered_laser_on = laser_on;
    s_rendered_result_valid = state.single_result_valid;
    s_rendered_unit = state.setting_distance_unit;
    if (laser_on) {
        s_laser_image = image_asset(s_root, &ui_single_laser_on, 170, 40);
    } else {
        s_laser_image = image_asset(s_root, &ui_single_laser_label, 170, 44);
        image_asset(s_root, &ui_single_laser_off, 211, 44);
    }

    s_crosshair_image = image_asset(s_root, &ui_single_crosshair, 83, 106);
    update_calibrated_crosshair(state);
    image_asset(s_root, &ui_single_level_track, 227, 93);
    s_level_marker = lv_obj_create(s_root);
    lv_obj_set_size(s_level_marker, 8, 8);
    lv_obj_set_pos(s_level_marker, 227, 139);
    lv_obj_set_style_radius(s_level_marker, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_level_marker, 0, 0);
    lv_obj_set_style_bg_opa(s_level_marker, LV_OPA_COVER, 0);

    s_result_dot = lv_obj_create(s_root);
    lv_obj_set_size(s_result_dot, 4, 4); lv_obj_set_pos(s_result_dot, 118, 141);
    lv_obj_set_style_radius(s_result_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_result_dot, 0, 0);
    lv_obj_set_style_bg_color(s_result_dot, lv_color_hex(kRed), 0);
    lv_obj_set_style_bg_opa(s_result_dot, LV_OPA_COVER, 0);

    s_value_panel = lv_obj_create(s_root);
    lv_obj_set_size(s_value_panel, 82, 30); lv_obj_set_pos(s_value_panel, 77, 163);
    style_screen(s_value_panel, 0x000000);
    lv_obj_set_style_radius(s_value_panel, 5, 0);
    lv_obj_set_style_bg_opa(s_value_panel, LV_OPA_80, 0);
    s_value = text(s_value_panel, "", 0, 0, kWhite);
    lv_obj_center(s_value);
    if (!state.single_result_valid) {
        lv_obj_add_flag(s_result_dot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_value_panel, LV_OBJ_FLAG_HIDDEN);
    }
    s_status = text(s_root, "", 4, 215, kWhite);
    s_error = text(s_root, "", 4, 215, kRed);

    lv_obj_t *bottom = overlay_bar(s_root, 239, 45);
    image_asset(bottom, &ui_single_bottom_back, 9, 10);
    s_center_action_image = image_asset(bottom,
        state.single_result_valid ? &ui_single_bottom_save : &ui_single_bottom_measure, 85, 10);
    image_asset(bottom, &ui_single_bottom_history, 161, 10);

    transparent_hitbox(s_root, 0, 36, 70, 35, event_single, 0);
    transparent_hitbox(s_root, 88, 36, 70, 35, event_single, 3);
    transparent_hitbox(s_root, 160, 36, 80, 35, event_single, 4);
    transparent_hitbox(s_root, 0, 239, 80, 45, event_single, 5);
    transparent_hitbox(s_root, 80, 239, 80, 45, event_single, 1);
    transparent_hitbox(s_root, 160, 239, 80, 45, event_single, 2);
}

void refresh_history_slots();

void storage_row(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    const uint8_t slot = static_cast<uint8_t>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    if (slot >= kHistorySlotCount) return;
    const uint8_t index = s_history_record_index[slot];
    if (index == 255 || !s_ui_state || index >= s_ui_state->measurement_record_visible) return;
    if (s_page == Page::RECORD_DELETE) {
        s_record_selection_mask ^= (1u << index);
        refresh_history_slots();
    } else {
        s_record_selected = index;
        go(Page::RECORD_DETAIL);
    }
}

bool load_record_thumbnail(const DeviceUiMeasurementRecord &record, uint8_t slot)
{
    constexpr int thumb_width = 50;
    constexpr int thumb_height = 60;
    if (slot >= DEVICE_UI_RECORD_CAPACITY || !s_thumbnail_pixels) return false;
    // 同槽位同记录:已有像素,直接返回
    if (s_thumbnail_record_id[slot] == record.id && s_thumbnail_dsc[slot].data)
        return s_thumbnail_loaded[slot];
    lv_color_t *destination = s_thumbnail_pixels + static_cast<size_t>(slot) * thumb_width * thumb_height;
    lv_img_dsc_t &descriptor = s_thumbnail_dsc[slot];
    descriptor.header.always_zero = 0;
    descriptor.header.w = thumb_width;
    descriptor.header.h = thumb_height;
    descriptor.header.cf = LV_IMG_CF_TRUE_COLOR;
    descriptor.data_size = thumb_width * thumb_height * sizeof(lv_color_t);
    descriptor.data = reinterpret_cast<const uint8_t *>(destination);
    s_thumbnail_record_id[slot] = record.id;
    s_thumbnail_loaded[slot] = false;

    // 全局缓存(按记录 id):滚动换槽位时避免重复读 SD 解码
    if (s_thumb_cache) {
        for (uint8_t i = 0; i < kThumbCacheCount; ++i) {
            ThumbCacheEntry &entry = s_thumb_cache[i];
            if (entry.valid && entry.id == record.id) {
                std::memcpy(destination, entry.pixels,
                            thumb_width * thumb_height * sizeof(lv_color_t));
                s_thumbnail_loaded[slot] = true;
                return true;
            }
        }
    }
    if (!record.camera_frame_saved || !s_record_decode || !s_thumbnail_pixels ||
        !s_cb.read_record_photo_rgb565 ||
        !s_cb.read_record_photo_rgb565(record.id, s_record_decode,
                                       CAMERA_JPEG_OUTPUT_PIXEL_COUNT)) return false;
    lv_color_t *cache_dst = nullptr;
    if (s_thumb_cache) {
        ThumbCacheEntry &entry = s_thumb_cache[s_thumb_cache_next];
        s_thumb_cache_next = static_cast<uint8_t>((s_thumb_cache_next + 1) % kThumbCacheCount);
        entry.id = record.id;
        entry.valid = true;
        cache_dst = entry.pixels;
    }
    for (int y = 0; y < thumb_height; ++y) {
        const int source_y = kCameraCropRows + y * kHeight / thumb_height;
        for (int x = 0; x < thumb_width; ++x) {
            const int source_x = kWidth - 1 - x * kWidth / thumb_width;
            const uint16_t pixel = swap16(s_record_decode[source_y * kWidth + source_x]);
            destination[y * thumb_width + x].full = pixel;
            if (cache_dst) cache_dst[y * thumb_width + x].full = pixel;
        }
    }
    s_thumbnail_loaded[slot] = true;
    return true;
}

void format_record_time(int64_t t_us, char *date, size_t date_size,
                        char *time, size_t time_size)
{
    const int64_t epoch_seconds = t_us / 1000000;
    if (epoch_seconds < 1609459200) {
        std::snprintf(date, date_size, "时间未同步");
        std::snprintf(time, time_size, "--:--");
        return;
    }
    const time_t timestamp = static_cast<time_t>(epoch_seconds);
    struct tm local = {};
    localtime_r(&timestamp, &local);
    std::strftime(date, date_size, "%Y-%m-%d", &local);
    std::strftime(time, time_size, "%H:%M", &local);
}

void record_select_event(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        s_record_selection_mask = 0;
        go(Page::RECORD_DELETE);
    }
}

void record_delete_event(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED || !s_cb.delete_measurement_record) return;
    if (s_page == Page::RECORD_DELETE) {
        bool changed = false;
        const uint8_t count = std::min<uint8_t>(s_ui_state->measurement_record_visible,
                                                DEVICE_UI_RECORD_CAPACITY);
        for (uint8_t index = 0; index < count; ++index) {
            if (s_record_selection_mask & (1u << index)) {
                changed = s_cb.delete_measurement_record(
                    s_ui_state->measurement_records[index].id) || changed;
            }
        }
        if (changed) {
            s_record_selection_mask = 0;
            go(Page::STORAGE);
        }
    } else if (s_pending_delete_id && s_cb.delete_measurement_record(s_pending_delete_id)) {
        go(Page::STORAGE);
    }
}

// 更新某槽位的高亮/勾选样式(内容未变时也会调用)
static void update_slot_highlight(uint8_t slot, uint8_t record_index)
{
    if (!s_history_card[slot]) return;
    // 按键导航高亮:当前记录索引 == s_record_selected 时显示边框
    const bool key_selected = record_index == s_record_selected;
    lv_obj_set_style_border_width(s_history_card[slot], key_selected ? 2 : 0, 0);
    lv_obj_set_style_border_color(s_history_card[slot], lv_color_hex(kGreen), 0);
    if (s_history_box[slot]) {
        const bool selected = (s_record_selection_mask & (1u << record_index)) != 0;
        lv_obj_set_style_border_color(s_history_box[slot],
                                      lv_color_hex(selected ? 0x3662EC : kMuted), 0);
        lv_obj_set_style_bg_color(s_history_box[slot],
                                  lv_color_hex(selected ? 0x3662EC : 0x000000), 0);
        if (selected) lv_obj_clear_flag(s_history_check[slot], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_history_check[slot], LV_OBJ_FLAG_HIDDEN);
    }
}

void bind_history_slot(uint8_t slot, uint8_t record_index, uint8_t first)
{
    if (!s_ui_state || slot >= kHistorySlotCount || !s_history_card[slot]) return;
    const uint8_t count = std::min<uint8_t>(s_ui_state->measurement_record_visible,
                                            DEVICE_UI_RECORD_CAPACITY);
    // 卡片在内容坐标系中的位置随 first 平移,滚动到底时最后几条记录仍落在视口内
    const int card_y = 6 + (static_cast<int>(first) + slot) * 70;
    if (record_index >= count) {
        if (s_history_record_index[slot] == 255) return;
        s_history_record_index[slot] = 255;
        lv_obj_add_flag(s_history_card[slot], LV_OBJ_FLAG_HIDDEN);
        if (s_history_box[slot]) lv_obj_add_flag(s_history_box[slot], LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(s_history_card[slot], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_y(s_history_card[slot], card_y);
    if (s_history_box[slot]) {
        lv_obj_clear_flag(s_history_box[slot], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_y(s_history_box[slot], 29 + (static_cast<int>(first) + slot) * 70);
    }
    // 内容未变(滚动回同一记录):只更新高亮/勾选,跳过缩略图与文字重绑(防卡顿)
    if (s_history_record_index[slot] == record_index && s_thumbnail_loaded[slot]) {
        update_slot_highlight(slot, record_index);
        return;
    }
    s_history_record_index[slot] = record_index;

    const auto &record = s_ui_state->measurement_records[record_index];
    load_record_thumbnail(record, slot);
    lv_img_set_src(s_history_thumb[slot], &s_thumbnail_dsc[slot]);
    lv_obj_invalidate(s_history_thumb[slot]);

    const char *type_name = "单点测距";
    if (record.type == DeviceUiMeasurementRecord::Type::P2P) type_name = "P2P测量";
    else if (record.type == DeviceUiMeasurementRecord::Type::FLOORPLAN) type_name = "平面图";
    lv_label_set_text(s_history_type[slot], type_name);

    char value[32];
    if (record.type == DeviceUiMeasurementRecord::Type::FLOORPLAN)
        std::snprintf(value, sizeof(value), "编号 %lu",
                      static_cast<unsigned long>(record.floorplan_number));
    else if (s_ui_state->setting_distance_unit == static_cast<uint8_t>(DistanceUnit::METRES))
        std::snprintf(value, sizeof(value), "%.3f m",
                      record.distance_mm / 1000.0f);
    else
        std::snprintf(value, sizeof(value), "%ld mm",
                      static_cast<long>(record.distance_mm));
    lv_label_set_text(s_history_value[slot], value);

    char date[24], time[16];
    format_record_time(record.t_us, date, sizeof(date), time, sizeof(time));
    lv_label_set_text(s_history_date[slot], date);
    lv_label_set_text(s_history_time[slot], time);

    update_slot_highlight(slot, record_index);
}

// 按键导航定位:让高亮卡(s_record_selected)固定在视口内相对 70px 处,
// 并回写 s_history_first 供 refresh_history_slots 使用。每次 OK 逐格滚动,
// 越界(selected 回卷到 0)时滚回顶部。
static void scroll_to_selected(uint8_t count)
{
    if (!s_history_list || count == 0) return;
    count = std::min<uint8_t>(count, DEVICE_UI_RECORD_CAPACITY);
    if (s_record_selected >= count) s_record_selected = static_cast<uint8_t>(count - 1);
    int first = static_cast<int>(s_record_selected) - 1;
    if (first < 0) first = 0;
    if (count > kHistorySlotCount && first > count - kHistorySlotCount)
        first = count - kHistorySlotCount;
    if (first > count - 1) first = count - 1;
    s_history_first = static_cast<uint8_t>(first);
    s_history_key_mode = true;
    // 高亮卡槽位 = selected - first,卡片 y = 6+(first+slot)*70 → 高亮卡绝对 y = 6+selected*70
    // (与 first 无关)。视口顶 = 高亮卡 y - 70,使高亮卡停在视口内相对 70px 处。
    const int slot_y = 6 + static_cast<int>(s_record_selected) * 70;
    int target = slot_y - 70;
    if (target < 0) target = 0;
    int scroll_end = count * 70 + 6 - 203;
    if (scroll_end < 0) scroll_end = 0;
    if (target > scroll_end) target = scroll_end;
    lv_obj_update_layout(s_history_list);
    s_program_scroll_tick = lv_tick_get();
    lv_obj_scroll_to_y(s_history_list, target, LV_ANIM_OFF);
    s_history_key_mode = true;  // 编程滚动触发的事件会在短时间内被吞掉,标志保持
}

void refresh_history_slots()
{
    if (!s_history_list || !s_ui_state) return;
    const uint8_t count = std::min<uint8_t>(s_ui_state->measurement_record_visible,
                                            DEVICE_UI_RECORD_CAPACITY);
    int first;
    if (s_history_key_mode) {
        first = s_history_first;
        if (first < 0) first = 0;
        if (count > kHistorySlotCount && first > count - kHistorySlotCount)
            first = count - kHistorySlotCount;
        if (first > count - 1) first = count - 1;
        s_history_first = static_cast<uint8_t>(first);
    } else {
        int scroll_y = lv_obj_get_scroll_y(s_history_list);
        if (scroll_y < 0) scroll_y = 0;
        first = scroll_y / 70 - 1;
        if (first < 0) first = 0;
        if (count > kHistorySlotCount && first > count - kHistorySlotCount)
            first = count - kHistorySlotCount;
        s_history_first = static_cast<uint8_t>(first);
    }
    for (uint8_t slot = 0; slot < kHistorySlotCount; ++slot)
        bind_history_slot(slot, static_cast<uint8_t>(first + slot), static_cast<uint8_t>(first));
}

void history_scroll_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code != LV_EVENT_SCROLL && code != LV_EVENT_SCROLL_END) return;
    // 编程滚动(scroll_to_selected)后 100ms 内的事件由调用方负责刷新,直接吞掉
    if (lv_tick_get() - s_program_scroll_tick < 100) return;
    s_history_key_mode = false;  // 真实触摸/惯性滚动接管,按滚动位置推导
    const uint8_t count = std::min<uint8_t>(s_ui_state ? s_ui_state->measurement_record_visible : 0,
                                            DEVICE_UI_RECORD_CAPACITY);
    int scroll_y = lv_obj_get_scroll_y(s_history_list);
    if (scroll_y < 0) scroll_y = 0;
    int first = scroll_y / 70 - 1;
    if (first < 0) first = 0;
    if (count > kHistorySlotCount && first > count - kHistorySlotCount)
        first = count - kHistorySlotCount;
    // 拖动中仅在跨过整格(记录窗变化)或滚动停止时刷新,避免每帧全量重绑
    if (first != s_history_first || code == LV_EVENT_SCROLL_END) {
        s_history_first = static_cast<uint8_t>(first);
        refresh_history_slots();
    }
}

void build_record_list(const DeviceUiState &state, bool selecting)
{
    clear_screen();
    // 强制本次进入时全部槽位重新绑定(记录数据可能已变化,防止短路跳过重绑)
    for (uint8_t slot = 0; slot < kHistorySlotCount; ++slot)
        s_history_record_index[slot] = 255;
    add_header(state, selecting ? "选择测量记录" : "测量记录", static_cast<Page>(255));

    s_history_list = lv_obj_create(s_root);
    lv_obj_set_size(s_history_list, kWidth, 203);
    lv_obj_set_pos(s_history_list, 0, 36);
    style_screen(s_history_list, 0x000000);
    lv_obj_add_flag(s_history_list, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                                    LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_set_scroll_dir(s_history_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_history_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_all(s_history_list, 0, 0);
    lv_obj_add_event_cb(s_history_list, history_scroll_event, LV_EVENT_ALL, nullptr);

    const uint8_t count = std::min<uint8_t>(state.measurement_record_visible,
                                            DEVICE_UI_RECORD_CAPACITY);
    if (!state.sd_ready) {
        text(s_history_list, "存储卡未就绪", 64, 86, kRed);
    } else if (!count) {
        text(s_history_list, "暂无测量记录", 72, 86, kMuted);
    } else {
        // This invisible spacer gives LVGL the full scroll range while only
        // five reusable cards are kept in memory.
        lv_obj_t *spacer = lv_obj_create(s_history_list);
        lv_obj_remove_style_all(spacer);
        lv_obj_set_size(spacer, 1, std::max<int>(203, count * 70 + 6));
        lv_obj_set_pos(spacer, 0, 0);
        lv_obj_clear_flag(spacer, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

        for (uint8_t slot = 0; slot < kHistorySlotCount; ++slot) {
            lv_obj_t *card = lv_obj_create(s_history_list);
            s_history_card[slot] = card;
            lv_obj_set_size(card, selecting ? 201 : 211, 66);
            lv_obj_set_pos(card, selecting ? 25 : 15, 6 + slot * 70);
            lv_obj_set_style_bg_color(card, lv_color_hex(0x202326), 0);
            lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(card, 0, 0);
            lv_obj_set_style_radius(card, 5, 0);
            lv_obj_set_style_pad_all(card, 0, 0);
            lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(card, storage_row, LV_EVENT_CLICKED,
                                reinterpret_cast<void *>(static_cast<intptr_t>(slot)));

            load_record_thumbnail(state.measurement_records[std::min<uint8_t>(slot, count - 1)], slot);
            s_history_thumb[slot] = image_asset(card, &s_thumbnail_dsc[slot], 3, 3);
            lv_obj_set_style_radius(s_history_thumb[slot], 4, 0);
            lv_obj_set_style_clip_corner(s_history_thumb[slot], true, 0);
            s_history_type[slot] = text(card, "", 59, 6);
            s_history_value[slot] = text(card, "", 59, 27, 0xBAE0FF);
            s_history_date[slot] = text(card, "", 59, 47, kMuted);
            s_history_time[slot] = text(card, "", 0, 47, kMuted);
            lv_obj_align(s_history_time[slot], LV_ALIGN_BOTTOM_RIGHT, -5, -3);

            if (selecting) {
                s_history_box[slot] = lv_obj_create(s_history_list);
                lv_obj_set_size(s_history_box[slot], 20, 20);
                lv_obj_set_pos(s_history_box[slot], 2, 29 + slot * 70);
                lv_obj_set_style_radius(s_history_box[slot], 3, 0);
                lv_obj_set_style_border_width(s_history_box[slot], 1, 0);
                lv_obj_set_style_bg_opa(s_history_box[slot], LV_OPA_COVER, 0);
                lv_obj_clear_flag(s_history_box[slot], LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(s_history_box[slot], LV_OBJ_FLAG_CLICKABLE);
                lv_obj_add_event_cb(s_history_box[slot], storage_row, LV_EVENT_CLICKED,
                                    reinterpret_cast<void *>(static_cast<intptr_t>(slot)));
                s_history_check[slot] = text(s_history_box[slot], LV_SYMBOL_OK, 0, 0,
                                             kWhite, &lv_font_montserrat_14);
                lv_obj_center(s_history_check[slot]);
            }
        }
        refresh_history_slots();
    }

    // 进入列表时定位到当前选中项(rebuild 后立即定位,高亮卡固定在视口内)
    if (!selecting && s_history_list && count > 0) {
        scroll_to_selected(count);
        refresh_history_slots();
    }

    lv_obj_t *bottom = overlay_bar(s_root, 239, 45);
    image_asset(bottom, selecting ? &ui_single_bottom_delete : &ui_single_bottom_select, 48, 10);
    image_asset(bottom, &ui_single_bottom_back, 128, 10);
    transparent_hitbox(s_root, 0, 239, 120, 45,
                       selecting ? record_delete_event : record_select_event, 0);
    transparent_hitbox(s_root, 120, 239, 120, 45, event_go,
                       static_cast<intptr_t>(selecting ? Page::STORAGE : s_storage_return));
}

void build_storage(const DeviceUiState &state) { build_record_list(state, false); }

void build_record_detail(const DeviceUiState &state)
{
    clear_screen();
    if (!state.measurement_record_visible) {
        text(s_root, "记录不存在", 72, 120, kRed);
        transparent_hitbox(s_root, 0, 0, kWidth, kHeight, event_go,
                           static_cast<intptr_t>(Page::STORAGE));
        return;
    }
    s_record_selected = std::min<uint8_t>(s_record_selected,
                                           state.measurement_record_visible - 1);
    const auto &record = state.measurement_records[s_record_selected];
    const bool photo_ok = record.camera_frame_saved && s_record_decode &&
        s_cb.read_record_photo_rgb565 &&
        s_cb.read_record_photo_rgb565(record.id, s_record_decode,
                                      CAMERA_JPEG_OUTPUT_PIXEL_COUNT);
    if (photo_ok) {
        copy_camera_crop_to_lvgl(s_record_decode, 0);
        image_asset(s_root, &s_camera_dsc, 0, 0);
    }
    add_header(state, "测量记录", Page::STORAGE);
    lv_obj_t *dot = lv_obj_create(s_root);
    lv_obj_set_size(dot, 4, 4); lv_obj_set_pos(dot, 118, 141);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(kRed), 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_t *panel = lv_obj_create(s_root);
    lv_obj_set_size(panel, 82, 30); lv_obj_set_pos(panel, 77, 163);
    style_screen(panel, 0x000000); lv_obj_set_style_radius(panel, 5, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_80, 0);
    char distance[28];
    if (state.setting_distance_unit == static_cast<uint8_t>(DistanceUnit::METRES))
        std::snprintf(distance, sizeof(distance), "%.3f m", record.distance_mm / 1000.0f);
    else
        std::snprintf(distance, sizeof(distance), "%ldmm", static_cast<long>(record.distance_mm));
    lv_obj_t *value = text(panel, distance, 0, 0); lv_obj_center(value);
    lv_obj_t *bottom = overlay_bar(s_root, 239, 45);
    image_asset(bottom, &ui_single_bottom_delete, 88, 10);
    s_pending_delete_id = record.id;
    transparent_hitbox(s_root, 40, 239, 160, 45, record_delete_event, 0);
}

void build_record_delete(const DeviceUiState &state) { build_record_list(state, true); }

void setting_event(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    const intptr_t item = reinterpret_cast<intptr_t>(lv_event_get_user_data(event));
    s_settings_index = static_cast<uint8_t>(item);
    if (item == 0) { if (s_cb.cycle_setting) s_cb.cycle_setting(2); s_rebuild = true; }  // 距离单位
    else if (item == 1) go(Page::WEB);            // 无线网络
    else if (item == 2) go(Page::IMU_CAL);        // 设备标定
    else if (item == 3) go(Page::SENSORS);        // 设备状态
    else if (item == 4) {                          // 解除绑定:清 NVS 并进入配对热点
        if (s_cb.unbind_pc) s_cb.unbind_pc();
        go(Page::WEB);
    }
}

void build_settings(const DeviceUiState &state)
{
    clear_screen();
    // 设计稿背景为纯黑,与主菜单一致(asset 由白色形状 recolor 上色)
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    // 顶部:返回箭头 + 标题 + 状态图标(与主菜单同位置)
    lv_obj_t *back_img = image_asset(s_root, &ui_settings_back, 8, 6);
    lv_obj_set_style_img_recolor(back_img, lv_color_hex(0xACCCE5), 0);
    lv_obj_set_style_img_recolor_opa(back_img, LV_OPA_COVER, 0);
    lv_obj_t *title_img = image_asset(s_root, &ui_settings_title, 42, 5);
    lv_obj_set_style_img_recolor(title_img, lv_color_hex(0xEFEFEF), 0);
    lv_obj_set_style_img_recolor_opa(title_img, LV_OPA_COVER, 0);
    // WiFi 图标:与电脑连接成功后才显示(与其它页面一致)
    if (state.pc_link_connected) {
        lv_obj_t *wifi = image_asset(s_root, &ui_menu_wifi, 163, 6);
        lv_obj_set_style_img_recolor(wifi, lv_color_hex(0xEFEFEF), 0);
        lv_obj_set_style_img_recolor_opa(wifi, LV_OPA_COVER, 0);
    }
    add_battery(s_root, 193, 10, state);
    transparent_hitbox(s_root, 0, 0, 44, 36, event_go, static_cast<intptr_t>(Page::MENU));

    // 5 行设置卡片(设计稿资产复用,选中项高亮)
    // 无线网络状态:连接成功优先显示"已连接",否则按 WiFi 开启状态显示
    const char *values[] = {
        state.setting_distance_unit == static_cast<uint8_t>(DistanceUnit::METRES) ? "米(m)" : "毫米(mm)",
        state.pc_link_connected ? "已连接" : (state.wifi_ready ? "未连接" : "未开启"),
        "", "实时", ""};
    const char *names[] = {"距离单位", "无线网络", "设备标定", "设备状态", "解除绑定"};
    for (int i = 0; i < 5; ++i) {
        const int y = 43 + i * 43;
        lv_obj_t *card = image_asset(s_root, &ui_settings_card, 8, y);
        const bool selected = (i == s_settings_index);
        lv_obj_set_style_img_recolor(card, lv_color_hex(selected ? 0x2A3550 : 0x151A1E), 0);
        lv_obj_set_style_img_recolor_opa(card, LV_OPA_COVER, 0);
        if (selected) {
            lv_obj_t *frame = lv_obj_create(s_root);
            lv_obj_remove_style_all(frame);
            lv_obj_set_size(frame, 224, 39);
            lv_obj_set_pos(frame, 8, y);
            lv_obj_set_style_border_color(frame, lv_color_hex(kGreen), 0);
            lv_obj_set_style_border_width(frame, 2, 0);
            lv_obj_set_style_radius(frame, 10, 0);
            lv_obj_clear_flag(frame, LV_OBJ_FLAG_SCROLLABLE);
        }
        text(s_root, names[i], 24, y + 10);
        const uint32_t value_color = (i == 1) ? (state.pc_link_connected ? kGreen :
                                     (state.wifi_ready ? kAmber : kMuted)) : kWhite;
        lv_obj_t *value = text(s_root, values[i], 0, y + 10, value_color);
        lv_obj_align_to(value, card, LV_ALIGN_RIGHT_MID, -16, 0);
        transparent_hitbox(s_root, 8, y, 224, 39, setting_event, i);
    }
}

void wifi_toggle(lv_event_t *event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && s_cb.request_web_toggle) {
        s_cb.request_web_toggle(); s_rebuild = true;
    }
}

void build_web(const DeviceUiState &state)
{
    clear_screen(); add_header(state, "无线网络", Page::SETTINGS);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    const char *status = state.web_busy ? "处理中" :
                         state.pc_link_connected ? "电脑已连接" :
                         state.pc_pairing_active ? "等待首次绑定" :
                         state.pc_binding_valid ? "等待电脑热点" :
                         state.wifi_ready ? "无线网络已开启" : "未开启";
    // 状态卡片(与设置页同款资产卡片)
    lv_obj_t *card = image_asset(s_root, &ui_settings_card, 8, 43);
    lv_obj_set_style_img_recolor(card, lv_color_hex(0x151A1E), 0);
    lv_obj_set_style_img_recolor_opa(card, LV_OPA_COVER, 0);
    lv_obj_t *status_text = text(s_root, status, 0, 53,
                                 state.web_busy ? kAmber : state.pc_link_connected ? kGreen : kWhite);
    lv_obj_align_to(status_text, card, LV_ALIGN_CENTER, 0, 0);
    if (state.pc_pairing_active) {
        text(s_root, "LASER-METER-SETUP", 47, 108, kWhite);
        text(s_root, "密码 12345678", 62, 136, kMuted);
    } else if (state.pc_binding_valid) {
        text(s_root, state.pc_name[0] ? state.pc_name : "已绑定电脑", 58, 116,
             state.pc_link_connected ? kGreen : kMuted);
    }
    button(s_root, state.wifi_ready ? "关闭" : "开启", 35, 178, 170, 54,
           wifi_toggle, 0, state.wifi_ready ? kRed : kGreen);
}

void build_sensors(const DeviceUiState &state)
{
    clear_screen(); add_header(state, "设备状态", Page::SETTINGS);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    const char *names[] = {"摄像头", "激光器", "惯性测量", "存储卡", "电池"};
    const bool ok[] = {state.camera_ready, state.laser_ready, state.bno_ready, state.sd_ready,
                       state.battery_valid};
    for (int i = 0; i < 5; ++i) {
        const int y = 43 + i * 43;
        lv_obj_t *card = image_asset(s_root, &ui_settings_card, 8, y);
        lv_obj_set_style_img_recolor(card, lv_color_hex(0x151A1E), 0);
        lv_obj_set_style_img_recolor_opa(card, LV_OPA_COVER, 0);
        text(s_root, names[i], 24, y + 10, kMuted);
        if (i == 4) {
            // 电池显示电量百分比
            char battery[24];
            std::snprintf(battery, sizeof(battery), "%u%%", state.battery_percent);
            lv_obj_t *value = text(s_root, battery, 0, y + 10,
                                   state.battery_percent > 20 ? kGreen : kRed);
            lv_obj_align_to(value, card, LV_ALIGN_RIGHT_MID, -16, 0);
        } else {
            lv_obj_t *value = text(s_root, ok[i] ? "正常" : "异常", 0, y + 10,
                                   ok[i] ? kGreen : kRed);
            lv_obj_align_to(value, card, LV_ALIGN_RIGHT_MID, -16, 0);
        }
    }
}

void action_event(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    const intptr_t action = reinterpret_cast<intptr_t>(lv_event_get_user_data(event));
    if (action == 0 && s_cb.request_measure) s_cb.request_measure();
    else if (action == 1 && s_cb.begin_room_survey) s_cb.begin_room_survey(false);
    else if (action == 2 && s_cb.finish_room_survey) s_cb.finish_room_survey();
    else if (action == 3 && s_cb.undo_room_survey) s_cb.undo_room_survey();
    else if (action == 4) {
        // Cancelling a live room scan is an exit operation, not merely a
        // state update.  Stop the backend first, then leave the page so both
        // the callback and apply_page_hardware() revoke laser ownership.
        if (s_cb.cancel_room_survey) s_cb.cancel_room_survey();
        go(Page::MENU);
        return;
    }
    else if (action == 5 && s_cb.request_single_measure) s_cb.request_single_measure();
    else if (action == 6) {
        go(Page::MENU);
        return;
    }
    else if (action == 7 && s_cb.begin_room_survey) s_cb.begin_room_survey(false);
    s_rebuild = true;
}

void p2p_event(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    switch (reinterpret_cast<intptr_t>(lv_event_get_user_data(event))) {
    case 0: if (s_cb.p2p_trigger) s_cb.p2p_trigger(); break;
    case 1: s_storage_return = Page::MEASURE; go(Page::STORAGE); return;
    case 2: go(Page::MENU); return;
    case 3: if (s_cb.p2p_save) s_cb.p2p_save(); break;
    case 4: if (s_cb.p2p_reset) s_cb.p2p_reset(); break;
    case 5: if (s_cb.p2p_remeasure) s_cb.p2p_remeasure(0); break;
    case 6: if (s_cb.p2p_remeasure) s_cb.p2p_remeasure(1); break;
    }
    s_rebuild = true;
}

void build_measure(const DeviceUiState &state)
{
    clear_screen();
    s_camera_requested.store(true, std::memory_order_release);
    s_camera_view = lv_img_create(s_root);
    lv_img_set_src(s_camera_view, &s_camera_dsc);
    lv_obj_set_size(s_camera_view, kWidth, kHeight);
    lv_obj_set_pos(s_camera_view, 0, 0);
    add_header(state, "P2P测量", static_cast<Page>(255));
    s_crosshair_image = image_asset(s_root, &ui_single_crosshair, 83, 82);
    update_calibrated_crosshair(state);
    // 固定精确模式(三脚架姿态,硬件不支持移动扫描/手持切换)
    text(s_root, "精确模式", 98, 39, kGreen);

    char line[64];
    const bool complete = state.p2p_a_valid && state.p2p_b_valid && state.p2p_space_distance_m > 0.0f;
    if (complete) {
        lv_obj_t *result = overlay_bar(s_root, 48, 137);
        text(result, "A <-> B", 91, 5, kWhite);
        std::snprintf(line, sizeof(line), "%.3f m", state.p2p_space_distance_m);
        lv_obj_t *space = text(result, line, 0, 28, kGreen); lv_obj_align(space, LV_ALIGN_TOP_MID, 0, 28);
        std::snprintf(line, sizeof(line), "水平：%.3f m", state.p2p_horizontal_distance_m); text(result, line, 47, 64);
        std::snprintf(line, sizeof(line), "高差：%+.3f m", state.p2p_height_diff_m); text(result, line, 47, 88);
        const char *quality = state.p2p_motion_risk == 0 ? "稳定" :
                              state.p2p_motion_risk == 1 ? "一般" : "平移风险";
        std::snprintf(line, sizeof(line), "%s  %.1fdeg  %ums", quality,
                      state.p2p_relative_angle_deg,
                      static_cast<unsigned>(state.p2p_imu_sync_error_ms));
        text(result, line, 36, 111, state.p2p_motion_risk == 0 ? kGreen : kAmber);
        button(s_root, "点A", 5, 37, 54, 30, p2p_event, 5, kGreen);
        button(s_root, "点B", 181, 37, 54, 30, p2p_event, 6, kGreen);
        button(s_root, "重新测量", 68, 194, 104, 36, p2p_event, 4, kPanel);
    } else {
        lv_obj_t *prompt = overlay_bar(s_root, 178, 52);
        const char *message = "请测量A点";
        if (state.p2p_stage == P2pStage::AIM_A) message = "激光已开启，再按一次测量A点";
        else if (state.p2p_stage == P2pStage::MEASURE_A) message = "正在测量A点...";
        else if (state.p2p_stage == P2pStage::WAIT_B) message = "[OK] 点A  请瞄准点B";
        else if (state.p2p_stage == P2pStage::AIM_B) message = "激光已开启，再按一次测量B点";
        else if (state.p2p_stage == P2pStage::MEASURE_B) message = "正在测量B点...";
        else if (state.p2p_stage == P2pStage::ERROR) message = "测量失败，请重试";
        lv_obj_t *label = text(prompt, message, 0, 0, state.p2p_stage == P2pStage::ERROR ? kRed : kWhite);
        lv_obj_center(label);
        if (state.p2p_a_valid) button(s_root, "OK A", 6, 37, 54, 30, p2p_event, 5, kGreen);
        if (state.p2p_b_valid) button(s_root, "OK B", 180, 37, 54, 30, p2p_event, 6, kGreen);
    }
    lv_obj_t *bottom = overlay_bar(s_root, 239, 45);
    text(bottom, "历史", 18, 14);
    text(bottom, complete ? (state.p2p_stage == P2pStage::SAVED ? "已保存" : "保存") : "测量",
         99, 14, complete ? kGreen : kAmber);
    text(bottom, "返回", 180, 14);
    transparent_hitbox(s_root, 0, 239, 80, 45, p2p_event, 1);
    transparent_hitbox(s_root, 80, 239, 80, 45, p2p_event, complete ? 3 : 0);
    transparent_hitbox(s_root, 160, 239, 80, 45, p2p_event, 2);
}

void build_room(const DeviceUiState &state)
{
    clear_screen(); add_header(state, "平面图扫描", Page::MENU);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    char line[80];
    // 信息区统一卡片(与设置页同款资产卡片)
    lv_obj_t *card = image_asset(s_root, &ui_settings_card, 8, 43);
    lv_obj_set_style_img_recolor(card, lv_color_hex(0x151A1E), 0);
    lv_obj_set_style_img_recolor_opa(card, LV_OPA_COVER, 0);
    if (state.room_complete) {
        text(s_root, LV_SYMBOL_OK, 111, 48, kGreen, &lv_font_montserrat_14);
        text(s_root, "扫描数据已保存", 60, 77, kGreen);
        std::snprintf(line, sizeof(line), "有效 %u  无效 %u", state.room_valid_count,
                      state.room_invalid_count);
        text(s_root, line, 52, 105, kMuted);
        if (state.room_scan_file[0]) {
            std::snprintf(line, sizeof(line), "文件 %.35s", state.room_scan_file);
            text(s_root, line, 12, 132, kWhite);
        }
        const char *upload = "保存在SD卡，等待电脑连接";
        uint32_t upload_color = kAmber;
        if (state.room_upload_state == RoomUploadState::UPLOADING) {
            upload = "正在上传到电脑…";
        } else if (state.room_upload_state == RoomUploadState::UPLOADED) {
            upload = "已上传电脑，等待PC处理";
            upload_color = kGreen;
        } else if (state.room_upload_state == RoomUploadState::RETRYING) {
            upload = state.pc_link_connected ? "上传失败，正在重试…" : "上传待重试，请开启PC服务";
            upload_color = kRed;
        } else if (state.room_upload_state == RoomUploadState::QUEUED && state.pc_link_connected) {
            upload = "已进入上传队列";
        }
        text(s_root, upload, 18, 159, upload_color);
        button(s_root, "返回菜单", 8, 204, 108, 54, action_event, 6, kMuted);
        button(s_root, "重新扫描", 124, 204, 108, 54, action_event, 7, kGreen);
        return;
    }
    const bool finish_ready = state.room_scan_coverage_complete && state.room_valid_count >= 30;
    std::snprintf(line, sizeof(line), "角度 %+.1f° / 360°", state.room_current_angle_deg);
    text(s_root, line, 20, 52, finish_ready ? kGreen : kWhite);
    std::snprintf(line, sizeof(line), "有效 %u  无效 %u", state.room_valid_count, state.room_invalid_count);
    text(s_root, line, 20, 80, kMuted);
    const char *motion = state.room_motion_risk >= 2 ? "高" : state.room_motion_risk == 1 ? "中" : "低";
    std::snprintf(line, sizeof(line), "俯仰 %+.1f°  平移风险 %s", state.room_pitch_deg, motion);
    text(s_root, line, 20, 108,
         !state.room_pose_ready ? kRed : state.room_motion_risk >= 2 ? kRed :
         state.room_motion_risk == 1 ? kAmber : kGreen);
    if (state.room_complete && state.room_scan_file[0]) {
        std::snprintf(line, sizeof(line), "已保存 %s", state.room_scan_file);
        text(s_root, line, 20, 136, kGreen);
    } else {
        text(s_root, state.room_pose_ready ? "固定测站，俯仰由IMU自动补偿" : "等待IMU姿态数据", 20, 136,
             state.room_pose_ready ? kMuted : kRed);
    }
    if (!state.room_active) button(s_root, "开始扫描", 35, 174, 170, 55, action_event, 1, kGreen);
    else {
        const char *scan_status = finish_ready ? "已满一周，可以结束" :
                                  state.room_scan_coverage_complete ? "有效点 < 30" :
                                  "正在连续采集…";
        text(s_root, scan_status, finish_ready ? 25 : 34, 164,
             finish_ready ? kGreen : kAmber);
        button(s_root, "取消", 8, 204, 94, 54, action_event, 4, kRed);
        button(s_root, "结束保存", 138, 204, 94, 54, action_event, 2, kGreen);
    }
}

void imu_event(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    if (s_imu_cal_step == 0 && s_cb.begin_imu_calibration && !s_cb.begin_imu_calibration()) return;
    if (s_imu_cal_step < 13 && (!s_cb.confirm_imu_calibration ||
        s_cb.confirm_imu_calibration(s_imu_cal_step))) ++s_imu_cal_step;
    if (s_imu_cal_step >= 13 && s_cb.finish_imu_calibration) s_cb.finish_imu_calibration();
    s_rebuild = true;
}

void build_imu_cal(const DeviceUiState &state)
{
    clear_screen(); add_header(state, "设备标定", Page::SETTINGS);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    lv_obj_t *card = image_asset(s_root, &ui_settings_card, 8, 92);
    lv_obj_set_style_img_recolor(card, lv_color_hex(0x151A1E), 0);
    lv_obj_set_style_img_recolor_opa(card, LV_OPA_COVER, 0);
    char line[48]; std::snprintf(line, sizeof(line), "步骤 %u / 13", std::min<unsigned>(s_imu_cal_step + 1, 13));
    text(s_root, line, 74, 102);
    text(s_root, "按提示放置设备后确认", 42, 130, kMuted);
    button(s_root, s_imu_cal_step >= 13 ? "完成" : "确认", 35, 178, 170, 58, imu_event, 0, kGreen);
}

void build_camera(const DeviceUiState &state)
{
    clear_screen(); s_camera_requested.store(true, std::memory_order_release);
    s_camera_view = lv_img_create(s_root); lv_img_set_src(s_camera_view, &s_camera_dsc);
    lv_obj_set_size(s_camera_view, kWidth, kHeight); lv_obj_set_pos(s_camera_view, 0, 0);
    add_header(state, "相机", Page::MENU);
    lv_obj_t *hint_panel = overlay_bar(s_root, 226, 58);
    s_camera_capture_status = text(hint_panel, "按OK拍照", 20, 8, kWhite);
    if (state.last_photo[0]) {
        char line[80];
        std::snprintf(line, sizeof(line), "已保存 %s", state.last_photo);
        lv_label_set_text(s_camera_capture_status, line);
    } else if (!state.sd_ready) {
        lv_label_set_text(s_camera_capture_status, "SD卡不可用");
        lv_obj_set_style_text_color(s_camera_capture_status, lv_color_hex(kRed), 0);
    }
    button(s_root, s_zoom ? "2x" : "1x", 184, 166, 50, 50, event_single, 3);
}

void rebuild_page(const DeviceUiState &state)
{
    switch (s_page) {
    case Page::MENU: build_menu(state); break;
    case Page::SINGLE: build_single(state); break;
    case Page::STORAGE: build_storage(state); break;
    case Page::RECORD_DETAIL: build_record_detail(state); break;
    case Page::RECORD_DELETE: build_record_delete(state); break;
    case Page::SETTINGS: build_settings(state); break;
    case Page::WEB: build_web(state); break;
    case Page::SENSORS: build_sensors(state); break;
    case Page::MEASURE: build_measure(state); break;
    case Page::ROOM: build_room(state); break;
    case Page::IMU_CAL: build_imu_cal(state); break;
    case Page::CAMERA: build_camera(state); break;
    }
}

void apply_page_hardware(Page page)
{
    const bool single_active = page == Page::SINGLE;
    const bool p2p_active = page == Page::MEASURE;
    // SINGLE owns the laser through its two-stage safety state machine:
    // entering the live-preview page must never emit laser-enable commands.
    const bool laser_active = false;
    if (page == s_hardware_page) {
        // A serial/API emergency stop can disable the backend without changing
        // the visible page. Reassert ownership when that measurement page is
        // rebuilt so its controls never remain connected to an inactive laser.
        if (laser_active && s_cb.set_laser_active) s_cb.set_laser_active(true);
        return;
    }
    if (s_hardware_page == Page::ROOM && page != Page::ROOM &&
        s_ui_state && s_ui_state->room_active && s_cb.cancel_room_survey) {
        s_cb.cancel_room_survey();
    }
    if (s_cb.set_single_page_active) s_cb.set_single_page_active(single_active);
    if (s_cb.set_p2p_page_active) s_cb.set_p2p_page_active(p2p_active);
    if (s_cb.set_laser_active) s_cb.set_laser_active(laser_active);
    s_camera_requested.store(page == Page::SINGLE || page == Page::MEASURE || page == Page::CAMERA,
                             std::memory_order_release);
    s_hardware_page = page;
}

bool camera_acquire(const uint16_t **pixels, uint32_t *sequence)
{
    bool available = false;
    portENTER_CRITICAL(&s_camera_mux);
    if (s_camera_front >= 0 && s_camera_reading < 0) {
        s_camera_reading = s_camera_front; *pixels = s_camera[s_camera_reading];
        *sequence = s_camera_sequence; available = true;
    }
    portEXIT_CRITICAL(&s_camera_mux);
    return available;
}

void camera_release()
{
    portENTER_CRITICAL(&s_camera_mux); s_camera_reading = -1; portEXIT_CRITICAL(&s_camera_mux);
}

void camera_task(void *)
{
    while (true) {
        if (!s_camera_requested.load(std::memory_order_relaxed) || !s_cb.read_camera_rgb565) {
            vTaskDelay(pdMS_TO_TICKS(20)); continue;
        }
        int target = -1;
        portENTER_CRITICAL(&s_camera_mux);
        for (int i = 0; i < 2; ++i) if (i != s_camera_front && i != s_camera_reading) { target = i; break; }
        if (s_camera_front < 0 && s_camera_reading < 0) target = 0;
        portEXIT_CRITICAL(&s_camera_mux);
        if (target < 0) { vTaskDelay(1); continue; }
        if (!s_cb.read_camera_rgb565(s_camera[target], CAMERA_JPEG_OUTPUT_PIXEL_COUNT)) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }
        portENTER_CRITICAL(&s_camera_mux); s_camera_front = target; ++s_camera_sequence; portEXIT_CRITICAL(&s_camera_mux);
        if (s_lvgl_task) xTaskNotifyGive(s_lvgl_task);
        vTaskDelay(1);
    }
}

void update_camera()
{
    if (!s_camera_view) return;
    const uint16_t *camera = nullptr; uint32_t sequence = 0;
    if (camera_acquire(&camera, &sequence)) {
        if (sequence != s_last_camera_sequence) {
            copy_camera_crop_to_lvgl(camera, s_zoom);
            s_last_camera_sequence = sequence; lv_obj_invalidate(s_camera_view);
        }
        camera_release();
    }
}

void update_single(const DeviceUiState &state)
{
    if (!s_value) return;
    // 长按测量"测量并自动保存":结果到达且尚未保存时自动保存一次
    if (s_auto_save_single_pending && state.single_result_valid &&
        state.single_state != SingleDistanceState::SAVED) {
        s_auto_save_single_pending = false;
        if (s_cb.single_save) s_cb.single_save();
    }
    if (state.single_state == SingleDistanceState::SAVED) s_auto_save_single_pending = false;
    const bool laser_on = state.single_state == SingleDistanceState::AIMING ||
                          state.single_state == SingleDistanceState::MEASURING;
    if (laser_on != s_rendered_laser_on || state.single_result_valid != s_rendered_result_valid ||
        state.setting_distance_unit != s_rendered_unit) {
        s_rebuild = true;
        return;
    }
    update_calibrated_crosshair(state);
    if (s_reference_image) {
        static const lv_img_dsc_t *const reference_assets[] = {
            &ui_single_ref_rear, &ui_single_ref_front, &ui_single_ref_tripod,
        };
        lv_img_set_src(s_reference_image,
            reference_assets[static_cast<uint8_t>(state.single_selected_reference)]);
        const bool rear_reference = state.single_selected_reference == DistanceReference::REAR;
        lv_obj_set_pos(s_reference_image, rear_reference ? 9 : 8,
                       rear_reference ? 45 : 43);
    }
    char value[32];
    if (state.single_result_valid) {
        if (state.setting_distance_unit == static_cast<uint8_t>(DistanceUnit::METRES))
            std::snprintf(value, sizeof(value), "%.3fm",
                          state.single_result.distance_mm / 1000.0f);
        else
            std::snprintf(value, sizeof(value), "%ldmm",
                          static_cast<long>(state.single_result.distance_mm));
    } else value[0] = '\0';
    lv_label_set_text(s_value, value);
    const char *message = "";
    switch (state.single_state) {
    case SingleDistanceState::MEASURING: message = "正在测量"; break;
    case SingleDistanceState::SAVING: message = "正在保存"; break;
    case SingleDistanceState::SAVED: message = "已保存"; break;
    default: break;
    }
    if (s_status) lv_label_set_text(s_status, message);
    if (s_error) {
        const char *error_message = "";
        if (state.single_state == SingleDistanceState::MEASURE_ERROR)
            error_message = "测量失败，请重试";
        else if (state.single_state == SingleDistanceState::SAVE_ERROR)
            error_message = "保存失败，结果仍保留";
        else if (state.single_state == SingleDistanceState::INIT_ERROR)
            error_message = "设备未就绪";
        lv_label_set_text(s_error, error_message);
    }
    const float pitch = state.fusion_pitch * 57.2957795f;
    if (s_level_marker) {
        const float clamped = std::clamp(pitch, -90.0f, 90.0f);
        const int y = 139 - static_cast<int>(clamped * (50.0f / 90.0f));
        lv_obj_set_y(s_level_marker, y);
        lv_obj_set_style_bg_color(s_level_marker,
            lv_color_hex(std::fabs(pitch) <= 3.0f ? kGreen : 0xBAE0FF), 0);
    }
}

// ---------------- 按键事件模型 ----------------
// 三个物理键 + 长短按,派生 6 个事件;每个页面按语义消费。
// 短按:measure/back/ok;长按(≥1.5s):long_measure/long_back/long_ok。
struct KeyEvent {
    bool measure = false;
    bool back = false;
    bool ok = false;
    bool long_measure = false;
    bool long_back = false;
    bool long_ok = false;
};

KeyEvent read_key_events(const DeviceUiState &state)
{
    KeyEvent ev;
    ev.measure = state.measure_press_count != s_last_measure;
    ev.back = state.back_press_count != s_last_back;
    ev.ok = state.ok_press_count != s_last_ok;
    ev.long_measure = state.long_measure_press_count != s_last_long_measure;
    ev.long_back = state.long_back_press_count != s_last_long_back;
    ev.long_ok = state.long_ok_press_count != s_last_long_ok;
    s_last_measure = state.measure_press_count;
    s_last_back = state.back_press_count;
    s_last_ok = state.ok_press_count;
    s_last_long_measure = state.long_measure_press_count;
    s_last_long_back = state.long_back_press_count;
    s_last_long_ok = state.long_ok_press_count;
    return ev;
}

static void keys_menu(const KeyEvent &ev);
static void keys_single(const KeyEvent &ev, const DeviceUiState &state);
static void keys_measure(const KeyEvent &ev, const DeviceUiState &state);
static void keys_room(const KeyEvent &ev, const DeviceUiState &state);
static void keys_camera(const KeyEvent &ev);
static void keys_storage(const KeyEvent &ev, const DeviceUiState &state);
static void keys_record_delete(const KeyEvent &ev, const DeviceUiState &state);
static void keys_record_detail(const KeyEvent &ev, const DeviceUiState &state);
static void keys_settings(const KeyEvent &ev);
static void keys_web(const KeyEvent &ev);
static void keys_sensors(const KeyEvent &ev);
static void keys_imu_cal(const KeyEvent &ev);

void handle_keys(const DeviceUiState &state)
{
    const KeyEvent ev = read_key_events(state);
    // 确认框优先:任何页面有确认框时,测量=确认,Back=取消
    if (s_confirm != 0) {
        if (ev.measure) {
            const uint8_t action = s_confirm;
            s_confirm = 0;
            if (action == 1 && s_cb.finish_room_survey) s_cb.finish_room_survey();
            else if (action == 2) {
                if (s_cb.cancel_room_survey) s_cb.cancel_room_survey();
                go(Page::MENU);  // 取消扫描后回菜单
            }
            else if (action == 3 && s_cb.request_photo) s_cb.request_photo();
            else if (action == 4) {
                if (s_ui_state && s_ui_state->measurement_record_visible &&
                    s_record_selected < s_ui_state->measurement_record_visible) {
                    if (s_cb.delete_measurement_record) {
                        s_cb.delete_measurement_record(
                            s_ui_state->measurement_records[s_record_selected].id);
                    }
                }
            }
            s_rebuild = true;
        } else if (ev.back) {
            s_confirm = 0;
            s_rebuild = true;
        }
        return;
    }
    switch (s_page) {
    case Page::MENU: keys_menu(ev); break;
    case Page::SINGLE: keys_single(ev, state); break;
    case Page::MEASURE: keys_measure(ev, state); break;
    case Page::ROOM: keys_room(ev, state); break;
    case Page::CAMERA: keys_camera(ev); break;
    case Page::STORAGE: keys_storage(ev, state); break;
    case Page::RECORD_DELETE: keys_record_delete(ev, state); break;
    case Page::RECORD_DETAIL: keys_record_detail(ev, state); break;
    case Page::SETTINGS: keys_settings(ev); break;
    case Page::WEB: keys_web(ev); break;
    case Page::SENSORS: keys_sensors(ev); break;
    case Page::IMU_CAL: keys_imu_cal(ev); break;
    }
}

// ---------- 各页面按键实现 ----------

void keys_menu(const KeyEvent &ev)
{
    if (ev.measure) designed_menu_enter(nullptr);
    else if (ev.ok) designed_menu_arrow(1, nullptr);
    else if (ev.long_measure) go(Page::SINGLE);       // 长按测量:直达单点
    else if (ev.long_back) go(Page::WEB);             // 长按Back:无线网络快捷
}

void keys_single(const KeyEvent &ev, const DeviceUiState &state)
{
    if (ev.measure) {
        if (state.single_result_valid) {
            if (s_cb.single_save) s_cb.single_save();
        } else if (s_cb.single_trigger) s_cb.single_trigger();
    } else if (ev.long_measure) {
        // 测量并自动保存:已有结果则保存;否则触发测量并置自动保存标记
        if (state.single_result_valid) {
            if (s_cb.single_save) s_cb.single_save();
        } else {
            if (s_cb.single_trigger) s_cb.single_trigger();
            s_auto_save_single_pending = true;
        }
    } else if (ev.ok) {
        if (s_cb.single_cycle_reference) s_cb.single_cycle_reference();
    } else if (ev.long_ok) {
        // 数码变焦 1x ↔ 2x
        s_zoom = static_cast<uint8_t>((s_zoom + 1) % 2);
        if (s_cb.set_camera_zoom) s_cb.set_camera_zoom(s_zoom ? 1 : 0);
        s_rebuild = true;
    } else if (ev.back) {
        if (state.single_state == SingleDistanceState::AIMING ||
            state.single_state == SingleDistanceState::MEASURING) {
            if (s_cb.single_cancel) s_cb.single_cancel();
            s_rebuild = true;
        } else {
            go(Page::MENU);
        }
    } else if (ev.long_back) {
        go(Page::MENU);
    }
}

void keys_measure(const KeyEvent &ev, const DeviceUiState &state)
{
    const bool complete = state.p2p_a_valid && state.p2p_b_valid && state.p2p_space_distance_m > 0.0f;
    if (ev.measure) {
        if (complete) {
            if (s_cb.p2p_save) s_cb.p2p_save();
        } else if (s_cb.p2p_trigger) s_cb.p2p_trigger();
    } else if (ev.long_measure) {
        // 重测当前点(按阶段判断 A 或 B)
        const uint8_t point = state.p2p_a_valid ? 1 : 0;
        if (s_cb.p2p_remeasure) s_cb.p2p_remeasure(point);
        s_rebuild = true;
    } else if (ev.ok) {
        if (s_cb.p2p_remeasure) s_cb.p2p_remeasure(0);  // 重测 A
    } else if (ev.long_ok) {
        if (s_cb.p2p_remeasure) s_cb.p2p_remeasure(1);  // 重测 B
    } else if (ev.back) {
        go(Page::MENU);
    } else if (ev.long_back) {
        go(Page::MENU);
    }
}

void keys_room(const KeyEvent &ev, const DeviceUiState &state)
{
    if (ev.measure) {
        if (state.room_active) {
            if (state.room_scan_coverage_complete && state.room_valid_count >= 30) {
                // 完成条件满足:直接完成并保存
                if (s_cb.finish_room_survey) s_cb.finish_room_survey();
            } else {
                // 未达条件:提示而非确认(结束保存按钮由触摸屏提供,条件不足时不结束)
                s_rebuild = true;
            }
        } else if (s_cb.begin_room_survey) {
            s_cb.begin_room_survey(false);  // 定点扫描(移动模式硬件不支持)
            s_rebuild = true;
        }
    } else if (ev.ok) {
        // 扫描中角度/有效点已同屏显示,OK 无额外切换;保留按键为刷新显示
        s_rebuild = true;
    } else if (ev.back) {
        if (state.room_active) {
            s_confirm = 2;  // 取消扫描确认
            s_rebuild = true;
        } else {
            go(Page::MENU);
        }
    } else if (ev.long_back) {
        if (s_cb.cancel_room_survey) s_cb.cancel_room_survey();
        go(Page::MENU);
    }
}

void keys_camera(const KeyEvent &ev)
{
    if (ev.measure) {
        if (s_cb.request_photo) s_cb.request_photo();
        if (s_camera_capture_status) {
            lv_label_set_text(s_camera_capture_status, "正在保存...");
            lv_obj_set_style_text_color(s_camera_capture_status, lv_color_hex(kAmber), 0);
        }
    } else if (ev.ok) {
        // 拍照仅支持高清模式,OK 键无操作(占位,便于未来扩展分辨率切换)
    } else if (ev.back) {
        go(Page::MENU);
    }
}

void keys_storage(const KeyEvent &ev, const DeviceUiState &state)
{
    if (ev.ok) {
        // 下移选中(槽位循环),逐格滚动列表;不重建页面(重建会丢失滚动位置)
        if (state.measurement_record_visible > 0) {
            s_record_selected = static_cast<uint8_t>(
                (s_record_selected + 1) % state.measurement_record_visible);
            scroll_to_selected(state.measurement_record_visible);
            refresh_history_slots();  // 按定位结果重绑槽位并更新高亮
        }
    } else if (ev.measure) {
        go(Page::RECORD_DETAIL);
    } else if (ev.long_measure) {
        go(Page::RECORD_DELETE);
    } else if (ev.back) {
        go(s_storage_return);
    }
}

void keys_record_delete(const KeyEvent &ev, const DeviceUiState &state)
{
    if (ev.ok) {
        // 切换勾选当前项
        if (state.measurement_record_visible > 0) {
            s_record_selection_mask ^= (1u << s_record_selected);
            s_rebuild = true;
        }
    } else if (ev.measure) {
        // 确认删除选中:复用现有删除回调需要逐条删除,这里通过重新进入删除流程
        if (s_record_selection_mask && s_cb.delete_measurement_record) {
            for (uint8_t i = 0; i < state.measurement_record_visible; ++i) {
                if (s_record_selection_mask & (1u << i)) {
                    s_cb.delete_measurement_record(state.measurement_records[i].id);
                }
            }
            s_record_selection_mask = 0;
            go(Page::STORAGE);
        }
    } else if (ev.back) {
        s_record_selection_mask = 0;
        go(Page::STORAGE);
    }
}

void keys_record_detail(const KeyEvent &ev, const DeviceUiState &state)
{
    if (ev.measure) {
        s_confirm = 4;  // 删除该记录确认
        s_rebuild = true;
    } else if (ev.ok) {
        // 查看照片大图(若有)——复用 CAMERA 页显示记录照片暂不支持,无操作占位
    } else if (ev.back) {
        go(Page::STORAGE);
    }
}

void keys_settings(const KeyEvent &ev)
{
    if (ev.ok) {
        s_settings_index = static_cast<uint8_t>((s_settings_index + 1) % 5);
        s_rebuild = true;
    } else if (ev.measure) {
        switch (s_settings_index) {
        case 0: if (s_cb.cycle_setting) s_cb.cycle_setting(2); break;  // 距离单位
        case 1: go(Page::WEB); break;                                   // 无线网络
        case 2: go(Page::IMU_CAL); break;                               // 设备标定
        case 3: go(Page::SENSORS); break;                               // 设备状态
        case 4:                                                         // 解除绑定
            if (s_cb.unbind_pc) s_cb.unbind_pc();
            go(Page::WEB);
            break;
        }
        s_rebuild = true;
    } else if (ev.back) {
        go(Page::MENU);
    }
}

void keys_web(const KeyEvent &ev)
{
    if (ev.measure) {
        if (s_cb.request_web_toggle) s_cb.request_web_toggle();
        s_rebuild = true;
    } else if (ev.back) {
        go(Page::SETTINGS);
    }
}

void keys_sensors(const KeyEvent &ev)
{
    if (ev.back) {
        go(Page::SETTINGS);
    }
    // OK 键无操作(设备状态页只读实时数据)
}

void keys_imu_cal(const KeyEvent &ev)
{
    if (ev.measure) {
        if (s_imu_cal_step == 0 && s_cb.begin_imu_calibration && !s_cb.begin_imu_calibration()) return;
        if (s_imu_cal_step < 13 && (!s_cb.confirm_imu_calibration || s_cb.confirm_imu_calibration(s_imu_cal_step))) ++s_imu_cal_step;
        if (s_imu_cal_step >= 13 && s_cb.finish_imu_calibration) s_cb.finish_imu_calibration();
        s_rebuild = true;
    } else if (ev.back) {
        // 标定步骤单向推进,Back 直接退出(回退会破坏标定状态机)
        go(Page::SETTINGS);
        s_imu_cal_step = 0;
    }
}

void lcd_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *pixels)
{
    const uint16_t width = static_cast<uint16_t>(area->x2 - area->x1 + 1);
    const uint16_t height = static_cast<uint16_t>(area->y2 - area->y1 + 1);
    uint16_t *wire = reinterpret_cast<uint16_t *>(pixels);
    for (size_t i = 0; i < static_cast<size_t>(width) * height; ++i) wire[i] = swap16(wire[i]);
    const esp_err_t err = nv3030b_lcd_present_rect(static_cast<uint16_t>(area->x1),
        static_cast<uint16_t>(area->y1), width, height, wire, static_cast<size_t>(width) * height);
    if (err != ESP_OK) ESP_LOGW(TAG, "LCD flush: %s", esp_err_to_name(err));
    lv_disp_flush_ready(drv);
}

void touch_read(lv_indev_drv_t *, lv_indev_data_t *data)
{
    uint16_t x = 0, y = 0; bool pressed = false;
    if (s_cb.read_touch) s_cb.read_touch(&x, &y, &pressed);
    data->point.x = std::min<uint16_t>(x, kWidth - 1);
    data->point.y = std::min<uint16_t>(y, kHeight - 1);
    data->state = pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

void lvgl_task(void *arg)
{
    DeviceUiState &state = *static_cast<DeviceUiState *>(arg);
    Page rendered = static_cast<Page>(255);
    bool web_state_seen = false;
    bool last_web_ready = false;
    bool last_web_busy = false;
    uint16_t last_record_count = UINT16_MAX;
    uint8_t last_battery_percent = UINT8_MAX;
    bool last_battery_valid = false;
    uint32_t last_photo_count = UINT32_MAX;
    P2pStage last_p2p_stage = static_cast<P2pStage>(255);
    bool last_p2p_a_valid = false;
    bool last_p2p_b_valid = false;
    float last_p2p_space_distance_m = -2.0f;
    uint16_t last_room_point_count = UINT16_MAX;
    bool last_room_pose_ready = false;
    uint8_t last_room_motion_risk = UINT8_MAX;
    int last_room_angle_bucket = -1000000;
    int last_room_pitch_bucket = -1000000;
    bool last_room_coverage = false;
    bool last_room_active = false;
    bool last_room_complete = false;
    RoomUploadState last_room_upload_state = static_cast<RoomUploadState>(255);
    bool last_pc_link_connected = false;
    int64_t last_tick_us = esp_timer_get_time();
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        const uint32_t elapsed_ms = static_cast<uint32_t>(std::max<int64_t>(1, (now_us - last_tick_us) / 1000));
        lv_tick_inc(elapsed_ms); last_tick_us = now_us;
        const int external_page = s_external_page_request.exchange(-1, std::memory_order_acq_rel);
        if (external_page >= 0) {
            // All page state and LVGL object lifetime changes remain owned by
            // this task. Console/action tasks only publish a request.
            go(static_cast<Page>(external_page));
        }
        if (s_cb.read_state) s_cb.read_state(&state);
        handle_keys(state);
        // WiFi 图标按连接状态显示:连接变化时全局重建(所有带顶栏的页面)
        if (state.pc_link_connected != last_pc_link_connected) {
            s_rebuild = true;
            last_pc_link_connected = state.pc_link_connected;
        }
        if ((s_page == Page::MENU || s_page == Page::SENSORS) &&
            (state.battery_percent != last_battery_percent ||
             state.battery_valid != last_battery_valid)) {
            s_rebuild = true;
        }
        last_battery_percent = state.battery_percent;
        last_battery_valid = state.battery_valid;
        if ((s_page == Page::STORAGE || s_page == Page::RECORD_DELETE) &&
            state.measurement_record_count != last_record_count) {
            s_rebuild = true;
        }
        last_record_count = state.measurement_record_count;
        if (s_page == Page::CAMERA && state.photo_count != last_photo_count) {
            s_rebuild = true;
        }
        last_photo_count = state.photo_count;
        if (s_page == Page::MEASURE &&
            (state.p2p_stage != last_p2p_stage ||
             state.p2p_a_valid != last_p2p_a_valid ||
             state.p2p_b_valid != last_p2p_b_valid ||
             state.p2p_space_distance_m != last_p2p_space_distance_m)) {
            s_rebuild = true;
        }
        last_p2p_stage = state.p2p_stage;
        last_p2p_a_valid = state.p2p_a_valid;
        last_p2p_b_valid = state.p2p_b_valid;
        last_p2p_space_distance_m = state.p2p_space_distance_m;
        if (s_page == Page::ROOM &&
            (state.room_point_count / 5 != last_room_point_count / 5 ||
             state.room_pose_ready != last_room_pose_ready ||
             state.room_motion_risk != last_room_motion_risk ||
             static_cast<int>(state.room_current_angle_deg / 3.0f) != last_room_angle_bucket ||
             static_cast<int>(state.room_pitch_deg) != last_room_pitch_bucket ||
             state.room_scan_coverage_complete != last_room_coverage ||
             state.room_active != last_room_active ||
             state.room_complete != last_room_complete ||
             state.room_upload_state != last_room_upload_state)) {
            s_rebuild = true;
        }
        last_room_point_count = state.room_point_count;
        last_room_pose_ready = state.room_pose_ready;
        last_room_motion_risk = state.room_motion_risk;
        last_room_angle_bucket = static_cast<int>(state.room_current_angle_deg / 3.0f);
        last_room_pitch_bucket = static_cast<int>(state.room_pitch_deg);
        last_room_coverage = state.room_scan_coverage_complete;
        last_room_active = state.room_active;
        last_room_complete = state.room_complete;
        last_room_upload_state = state.room_upload_state;
        if (s_page == Page::WEB) {
            if (!web_state_seen || state.wifi_ready != last_web_ready || state.web_busy != last_web_busy) {
                s_rebuild = true;
                web_state_seen = true;
                last_web_ready = state.wifi_ready;
                last_web_busy = state.web_busy;
            }
        } else {
            web_state_seen = false;
        }
        if (!state.startup_complete ||
            (lv_tick_get() - s_startup_shown_tick < 2000)) {
            if (!s_rendered_startup || s_rebuild) {
                build_startup(state);
                s_rendered_startup = true;
                s_startup_shown_tick = lv_tick_get();
                s_rebuild = false;
            }
        } else {
            if (s_rendered_startup) { s_rendered_startup = false; s_rebuild = true; }
            if (rendered != s_page || s_rebuild) {
                apply_page_hardware(s_page); rebuild_page(state); rendered = s_page; s_rebuild = false;
            }
            // 确认框提示条(测量=确认 Back=取消)
            if (s_confirm != 0 && s_confirm_bar == nullptr) {
                s_confirm_bar = overlay_bar(s_root, 182, 62);
                text(s_confirm_bar, "确认操作?", 12, 8, kWhite);
                text(s_confirm_bar, "测量=确认  Back=取消", 12, 30, kMuted);
            } else if (s_confirm == 0 && s_confirm_bar != nullptr) {
                lv_obj_del(s_confirm_bar);
                s_confirm_bar = nullptr;
            }
            if ((s_page == Page::SINGLE && !state.single_result_valid) ||
                (s_page == Page::MEASURE &&
                 !(state.p2p_a_valid && state.p2p_b_valid && state.p2p_space_distance_m > 0.0f)) ||
                s_page == Page::CAMERA) update_camera();
            if (s_page == Page::SINGLE) update_single(state);
            if (s_page == Page::MEASURE) update_calibrated_crosshair(state);
            update_battery(state);
        }
        lv_timer_handler();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(8));
    }
}

}  // namespace

esp_err_t device_ui_start(const DeviceUiCallbacks *callbacks)
{
    if (s_lvgl_task) return ESP_OK;
    if (!callbacks || !callbacks->read_state) return ESP_ERR_INVALID_ARG;
    s_cb = *callbacks;
    ESP_RETURN_ON_ERROR(nv3030b_lcd_init(), TAG, "LCD init");
    lv_init();
    s_draw_a = static_cast<lv_color_t *>(heap_caps_malloc(kWidth * kDrawRows * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_draw_b = static_cast<lv_color_t *>(heap_caps_malloc(kWidth * kDrawRows * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_camera_image = static_cast<lv_color_t *>(heap_caps_malloc(kWidth * kHeight * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_thumbnail_pixels = static_cast<lv_color_t *>(heap_caps_malloc(
        DEVICE_UI_RECORD_CAPACITY * 50 * 60 * sizeof(lv_color_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_thumb_cache = static_cast<ThumbCacheEntry *>(heap_caps_malloc(
        kThumbCacheCount * sizeof(ThumbCacheEntry), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_camera[0] = camera_jpeg_decoder_alloc_output(); s_camera[1] = camera_jpeg_decoder_alloc_output();
    s_record_decode = camera_jpeg_decoder_alloc_output();
    s_ui_state = static_cast<DeviceUiState *>(heap_caps_calloc(1, sizeof(DeviceUiState),
                                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!s_draw_a || !s_draw_b || !s_camera_image || !s_thumbnail_pixels ||
        !s_camera[0] || !s_camera[1] ||
        !s_record_decode || !s_ui_state) return ESP_ERR_NO_MEM;
    std::memset(s_camera_image, 0, kWidth * kHeight * sizeof(lv_color_t));
    s_camera_dsc.header.always_zero = 0; s_camera_dsc.header.w = kWidth; s_camera_dsc.header.h = kHeight;
    s_camera_dsc.header.cf = LV_IMG_CF_TRUE_COLOR; s_camera_dsc.data_size = kWidth * kHeight * sizeof(lv_color_t);
    s_camera_dsc.data = reinterpret_cast<const uint8_t *>(s_camera_image);
    lv_disp_draw_buf_init(&s_draw_buf, s_draw_a, s_draw_b, kWidth * kDrawRows);
    lv_disp_drv_init(&s_disp_drv); s_disp_drv.hor_res = kWidth; s_disp_drv.ver_res = kHeight;
    s_disp_drv.draw_buf = &s_draw_buf; s_disp_drv.flush_cb = lcd_flush; lv_disp_drv_register(&s_disp_drv);
    lv_indev_drv_init(&s_indev_drv); s_indev_drv.type = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = touch_read; lv_indev_drv_register(&s_indev_drv);
    // LVGL page construction and the Chinese font renderer have deep call
    // paths. Keep the large state snapshot out of the task stack and retain
    // generous PSRAM-backed stack headroom for event-driven page rebuilds.
    if (xTaskCreatePinnedToCoreWithCaps(lvgl_task, "device_lvgl", 32768, s_ui_state, 5, &s_lvgl_task, 1,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCoreWithCaps(camera_task, "camera_decode", 8192, nullptr, 6, &s_camera_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "LVGL 8.4 UI started: Camera Task -> LVGL Task -> LCD DMA");
    return ESP_OK;
}

static void request_external_page(Page page)
{
    s_external_page_request.store(static_cast<int>(page), std::memory_order_release);
    if (s_lvgl_task) xTaskNotifyGive(s_lvgl_task);
}

void device_ui_show_camera() { request_external_page(Page::CAMERA); }
void device_ui_show_single() { request_external_page(Page::SINGLE); }
void device_ui_show_p2p() { request_external_page(Page::MEASURE); }
void device_ui_show_menu() { request_external_page(Page::MENU); }
void device_ui_show_imu_calibration() { request_external_page(Page::IMU_CAL); }
