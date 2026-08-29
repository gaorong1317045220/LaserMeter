#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "single_distance.h"

enum class DeviceCheckState : uint8_t {
    PENDING = 0,
    PASS = 1,
    WARNING = 2,
    FAIL = 3,
};

enum class P2pStage : uint8_t {
    WAIT_A = 0,
    AIM_A,
    MEASURE_A,
    WAIT_B,
    AIM_B,
    MEASURE_B,
    COMPLETE,
    SAVING,
    SAVED,
    ERROR,
};

enum class RoomUploadState : uint8_t {
    NONE = 0,
    QUEUED,
    UPLOADING,
    UPLOADED,
    RETRYING,
};

struct DeviceUiMeasurementRecord {
    enum class Type : uint8_t { SINGLE = 0, P2P = 1, FLOORPLAN = 2 };
    uint32_t id;
    int64_t t_us;
    int32_t distance_mm;
    DistanceReference reference;
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    bool camera_frame_saved;
    char image_path[96];
    uint8_t quality;
    uint16_t measure_time_ms;
    int32_t raw_error_code;
    Type type;
    uint32_t floorplan_number;
    int32_t distance_a_mm;
    int32_t distance_b_mm;
    int32_t horizontal_mm;
    int32_t height_diff_mm;
    char image_path_b[96];
    int64_t timestamp_a_ms;
    int64_t timestamp_b_ms;
};

static constexpr size_t DEVICE_UI_RECORD_CAPACITY = 32;

struct DeviceUiState {
    bool startup_complete;
    uint8_t startup_pass_count;
    uint8_t startup_warning_count;
    uint8_t startup_fail_count;
    DeviceCheckState check_lcd;
    DeviceCheckState check_i2c;
    DeviceCheckState check_input;
    DeviceCheckState check_sd;
    DeviceCheckState check_camera;
    DeviceCheckState check_laser_uart;
    bool camera_ready;
    bool wifi_ready;
    bool web_busy;
    bool pc_pairing_active;
    bool pc_binding_valid;
    bool pc_link_connected;
    RoomUploadState room_upload_state;
    char pc_name[49];
    bool sd_ready;
    bool bno_ready;
    bool laser_ready;
    bool laser_busy;
    bool key_measure;
    bool key_back;
    bool key_ok;
    int32_t laser_mm;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float qi;
    float qj;
    float qk;
    float qr;
    float path_x;
    float path_y;
    float path_z;
    float fusion_vx;
    float fusion_vy;
    float fusion_yaw;
    float fusion_pitch;
    float fusion_roll;
    float fusion_confidence;
    bool fusion_stationary;
    uint16_t battery_adc_raw;
    float battery_adc_voltage;
    float battery_voltage;
    uint8_t battery_percent;
    bool battery_valid;
    uint8_t measure_point_count;
    uint32_t measure_session_id;
    bool room_active;
    bool room_complete;
    uint32_t room_session_id;
    uint16_t room_point_count;
    float room_last_segment_m;
    float room_open_perimeter_m;
    float room_closure_m;
    float room_area_xy_m2;
    float room_current_angle_deg;
    float room_rotation_deg;
    float room_pitch_deg;
    uint16_t room_valid_count;
    uint16_t room_invalid_count;
    bool room_pose_ready;
    uint8_t room_motion_risk;
    float room_max_linear_accel_mps2;
    bool room_scan_coverage_complete;
    char room_scan_file[48];
    bool room_dxf_saved;
    char room_dxf_file[48];
    float room_x[64];
    float room_y[64];
    float point_distance_m;
    float point1_x;
    float point1_y;
    float point1_z;
    float point2_x;
    float point2_y;
    float point2_z;
    P2pStage p2p_stage;
    bool p2p_a_valid;
    bool p2p_b_valid;
    int32_t p2p_distance_a_mm;
    int32_t p2p_distance_b_mm;
    int64_t p2p_timestamp_a_ms;
    int64_t p2p_timestamp_b_ms;
    float p2p_pitch_a_deg;
    float p2p_roll_a_deg;
    float p2p_yaw_a_deg;
    float p2p_pitch_b_deg;
    float p2p_roll_b_deg;
    float p2p_yaw_b_deg;
    float p2p_space_distance_m;
    float p2p_horizontal_distance_m;
    float p2p_height_diff_m;
    bool p2p_tripod_mode;
    uint8_t p2p_motion_risk;
    float p2p_relative_angle_deg;
    float p2p_max_linear_accel_mps2;
    float p2p_max_gyro_dps;
    uint16_t p2p_imu_sync_error_ms;
    char p2p_image_a[96];
    char p2p_image_b[96];
    uint32_t bno_count;
    uint32_t laser_count;
    uint32_t photo_count;
    uint32_t last_saved_record_id;
    SingleDistanceState single_state;
    SingleDistanceError single_error;
    DistanceReference single_selected_reference;
    DistanceReference single_measurement_reference;
    int64_t single_aim_deadline_us;
    bool single_result_valid;
    SingleDistanceResult single_result;
    uint8_t single_history_count;
    SingleDistanceResult single_history[SINGLE_DISTANCE_HISTORY_CAPACITY];
    uint16_t measurement_record_count;
    uint8_t measurement_record_visible;
    DeviceUiMeasurementRecord measurement_records[DEVICE_UI_RECORD_CAPACITY];
    bool setting_auto_save_single;
    bool setting_photo_on_measure;
    uint8_t setting_distance_unit;
    uint32_t measure_press_count;
    uint32_t back_press_count;
    uint32_t ok_press_count;
    // 长按(≥1.5s)事件计数:按住超过阈值并在释放后自增一次,
    // 与短按计数分离,由 UI 层按页面语义消费。
    uint32_t long_measure_press_count;
    uint32_t long_back_press_count;
    uint32_t long_ok_press_count;
    int64_t uptime_us;
    char last_photo[64];
    char last_error[96];
};

struct DeviceUiCallbacks {
    void (*read_state)(DeviceUiState *state);
    // Produces a 16-byte-aligned PSRAM RGB565_BE 240x320 portrait frame. The
    // UI displays its centred 240x284 slice (18 rows removed at each end).
    bool (*read_camera_rgb565)(uint16_t *pixels, size_t pixel_count);
    bool (*read_record_photo_rgb565)(uint32_t record_id, uint16_t *pixels,
                                     size_t pixel_count);
    bool (*read_touch)(uint16_t *x, uint16_t *y, bool *pressed);
    void (*set_camera_zoom)(uint8_t zoom_level);
    void (*set_single_page_active)(bool active);
    void (*single_trigger)();
    void (*single_cancel)();
    void (*single_save)();
    void (*single_cycle_reference)();
    // Legacy quick capture used by AI assist. Product single-distance uses the
    // state-machine callbacks above.
    void (*request_single_measure)();
    void (*request_measure)();
    void (*set_p2p_page_active)(bool active);
    void (*p2p_trigger)();
    void (*p2p_remeasure)(uint8_t point_index);
    void (*p2p_cycle_mode)();
    void (*p2p_reset)();
    void (*p2p_save)();
    void (*request_photo)();
    void (*request_web_toggle)();
    // Clear the saved PC hotspot binding and restart pairing mode.
    bool (*unbind_pc)();
    bool (*delete_measurement_record)(uint32_t id);
    void (*cycle_setting)(uint8_t category);
    // The backend owns the laser UART. The UI only publishes whether the
    // current page is an aiming/measurement mode.
    void (*set_laser_active)(bool active);
    bool (*begin_imu_calibration)();
    bool (*confirm_imu_calibration)(uint8_t step);
    bool (*finish_imu_calibration)();
    bool (*begin_room_survey)(bool moving_mode);
    bool (*finish_room_survey)();
    bool (*undo_room_survey)();
    void (*cancel_room_survey)();
};

esp_err_t device_ui_start(const DeviceUiCallbacks *callbacks);
void device_ui_show_camera();
void device_ui_show_single();
void device_ui_show_p2p();
void device_ui_show_menu();
void device_ui_show_imu_calibration();
