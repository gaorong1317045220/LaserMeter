#include <algorithm>
#include <set>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <initializer_list>
#include <string>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include "pin_config.h"
#include "nv3030b_lcd.h"
#include "device_ui.h"
#include "fusion_engine.h"
#include "fusion_config.h"
#include "web_ui_pages.h"
#include "camera_jpeg_decoder.h"
#include "app_settings.h"
#include "measurement_store.h"
#include "single_distance.h"

#include "esp_camera.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_camera_af.h"
#include "esp_attr.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_rom_crc.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "img_converters.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdmmc_cmd.h"

extern "C" int SCCB_Deinit(void);

static constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;
// BNO086 produces several 20 ms reports on this shared bus. At 100 kHz its
// packet reads can monopolise the mutex long enough for touch polling to miss
// an entire press; all fitted devices support the board's intended 400 kHz.
static constexpr uint32_t I2C_FREQ_HZ = 400000;
static constexpr int I2C_TIMEOUT_MS = 80;
// Waiting for ownership and performing a transfer are different failure
// modes.  A 256-byte BNO read can repeatedly win the mutex while reports are
// queued, so lower-priority UI/diagnostic clients need a longer acquisition
// window even though an individual hardware transaction must still finish
// within I2C_TIMEOUT_MS.
static constexpr int I2C_MUTEX_TIMEOUT_MS = 250;
static constexpr int I2C_PROBE_MUTEX_TIMEOUT_MS = 500;
static constexpr char SD_MOUNT_POINT[] = "/sdcard";
static constexpr char DATASET_CAPTURE_DIR[] = "/dataset_capture";
// Keep the local preview at VGA and crop its centre after decoding.  Scaling
// VGA down to QVGA discarded the one/few pixels occupied by a distant laser
// spot, making it disappear on the LCD.
static constexpr framesize_t CAMERA_LOCAL_FRAME_SIZE = FRAMESIZE_VGA;
// Allocate frame buffers for the OV5640 full resolution.  The live preview
// runs at VGA; still capture temporarily switches to QSXGA without rebuilding
// the SCCB/I2C camera instance.
static constexpr framesize_t CAMERA_STILL_FRAME_SIZE = FRAMESIZE_QSXGA;
static constexpr framesize_t CAMERA_WEB_FRAME_SIZE = CAMERA_LOCAL_FRAME_SIZE;
// esp32-camera uses a lower number for higher JPEG quality.  Quality 5 keeps
// substantially more VGA detail than the previous Q10 setting; the real-device performance
// guard remains 10 FPS with the ESP32-S3 SIMD decoder.
static constexpr int CAMERA_HTTP_JPEG_QUALITY = 5;
static constexpr int WIFI_AP_MAX_TX_POWER_QDBM = 32;  // 8 dBm, units are 0.25 dBm.
static constexpr size_t WEB_TEXT_FILE_MAX_BYTES = 64 * 1024;
// Wi-Fi/LwIP, HTTP task stacks and transfer buffers preferentially live in
// PSRAM.  Keep one full 64 KiB internal/DMA reserve available, but do not
// reject a healthy driver merely because its post-init headroom is below the
// former arbitrary 100 KiB guard (the current board measures about 92 KiB).
static constexpr size_t WIFI_MIN_INTERNAL_FREE = 64 * 1024;
static constexpr size_t WIFI_MIN_INTERNAL_LARGEST = 32 * 1024;
static constexpr char PC_PAIRING_SSID[] = "LASER-METER-SETUP";
static constexpr char PC_PAIRING_PASSWORD[] = "12345678";
static constexpr char PC_DEFAULT_HOTSPOT_SSID[] = "LASER-PC";
static constexpr char PC_DEFAULT_HOTSPOT_PASSWORD[] = "12345678";
static constexpr uint16_t PC_PAIRING_PORT = 8766;
static constexpr uint16_t PC_DEFAULT_SERVER_PORT = 8765;
static constexpr char PC_LINK_NVS_NAMESPACE[] = "pc_link";
// Give the PC browser enough time to submit its hostname/hotspot credentials.
// A phone-only setup can still use the fixed defaults after the two-minute
// fallback without requiring an HTTP server on the ESP32.
static constexpr uint32_t PC_PAIRING_AUTO_BIND_MS = 120000;
static constexpr uint32_t PC_LINK_MAX_PACKET = 2 * 1024 * 1024;

static void log_memory(const char *stage)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    ESP_LOGI("heap", "%s internal_free=%u largest=%u minimum=%u psram_free=%u",
             stage,
             static_cast<unsigned>(heap_caps_get_free_size(caps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(caps)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(caps)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
}

static bool wifi_memory_headroom_ok()
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    return heap_caps_get_free_size(caps) >= WIFI_MIN_INTERNAL_FREE &&
           heap_caps_get_largest_free_block(caps) >= WIFI_MIN_INTERNAL_LARGEST;
}

static bool s_i2c_ready = false;
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static bool s_uart_laser_ready = false;
static bool s_sd_mounted = false;
static sdmmc_card_t *s_sd_card = nullptr;
static bool s_wifi_ready = false;
static bool s_wifi_started = false;
static esp_netif_t *s_wifi_sta_netif = nullptr;
static esp_netif_t *s_wifi_ap_netif = nullptr;
enum class WifiControlOperation : uint8_t {
    INIT,
    STOP,
    CONFIGURE_AND_START,
    CONNECT,
    SET_POWER_SAVE,
    SET_MAX_TX_POWER,
    SET_BANDWIDTH,
};
struct WifiControlRequest {
    WifiControlOperation operation = WifiControlOperation::INIT;
    wifi_mode_t mode = WIFI_MODE_NULL;
    wifi_interface_t interface = WIFI_IF_STA;
    wifi_config_t config = {};
    bool has_config = false;
    wifi_ps_type_t power_save = WIFI_PS_NONE;
    int8_t max_tx_power = 0;
    wifi_bandwidth_t bandwidth = WIFI_BW_HT20;
};
static TaskHandle_t s_wifi_control_task = nullptr;
static SemaphoreHandle_t s_wifi_control_mutex = nullptr;
static SemaphoreHandle_t s_wifi_control_done = nullptr;
static WifiControlRequest s_wifi_control_request;
static esp_err_t s_wifi_control_result = ESP_FAIL;
static httpd_handle_t s_camera_httpd = nullptr;
static httpd_handle_t s_stream_httpd = nullptr;
static httpd_handle_t s_file_httpd = nullptr;
static volatile bool s_stream_stop_requested = false;
struct PcBinding {
    bool valid = false;
    char hotspot_ssid[33] = {};
    char hotspot_password[64] = {};
    char pc_name[49] = {};
    uint8_t station_mac[6] = {};
    uint16_t server_port = PC_DEFAULT_SERVER_PORT;
};
static PcBinding s_pc_binding;
static TaskHandle_t s_pc_link_task = nullptr;
static std::atomic<bool> s_pc_link_enabled{true};
static std::atomic<bool> s_pc_pairing_active{false};
static std::atomic<bool> s_pc_link_connected{false};
static std::atomic<bool> s_pc_binding_loaded{false};
static std::atomic<bool> s_pc_binding_save_pending{false};
static std::atomic<bool> s_pc_binding_save_complete{false};
static std::atomic<bool> s_pc_binding_save_ok{false};
static PcBinding s_pc_binding_to_save;
static SemaphoreHandle_t s_pc_upload_mutex = nullptr;
// 上传 FIFO:照片与扫描 CSV 共用。照片在拍照成功后入队,扫描在 room_survey_finish
// 入队;pc_link 连接后逐个上传,成功后弹出。失败保留队首重试。
static constexpr size_t PC_UPLOAD_QUEUE_DEPTH = 8;
static constexpr size_t PC_UPLOAD_PATH_LEN = 96;
static char s_pc_pending_upload[PC_UPLOAD_QUEUE_DEPTH][PC_UPLOAD_PATH_LEN] = {};
static size_t s_pc_upload_head = 0;
static size_t s_pc_upload_tail = 0;
static size_t s_pc_upload_count = 0;
static std::atomic<RoomUploadState> s_room_upload_state{RoomUploadState::NONE};
static void pc_link_start_once();
static void pc_link_queue_upload(const char *relative_path, bool drop_when_full = true);
static bool pc_link_clear_binding();
static bool cmd_pc_unbind();  // 由 action_task 异步执行解除绑定
static void pc_time_sync_apply(int64_t utc_ms, int16_t tz_min);
static bool pc_time_sync_load(int64_t *utc_ms, int16_t *tz_min);
static esp_err_t device_file_server_start();
static PcBinding pc_binding_snapshot();
static void process_pending_pc_binding_save();
static void process_pending_pc_unbind();
static void process_pending_time_sync_save();
static esp_err_t wifi_control_start_once();
static camera_jpeg_decoder_t *s_preview_decoder = nullptr;
static std::atomic<uint8_t> s_camera_zoom{0};
static uint8_t s_decoder_zoom = 0;
static bool s_camera_http_ready = false;
static SemaphoreHandle_t s_camera_mutex = nullptr;
static SemaphoreHandle_t s_i2c_mutex = nullptr;
static SemaphoreHandle_t s_i2c_init_mutex = nullptr;
static gpio_num_t s_laser_active_tx = PIN_LASER_TX;
static gpio_num_t s_laser_active_rx = PIN_LASER_RX;
static int s_laser_active_baud = LASER_BAUD;
static uint8_t s_bno_addr = I2C_ADDR_BNO086;
static FusionEngine s_fusion;
static MeasurementStore s_measurement_store;
static SingleDistanceSession s_single_distance;
static AppSettings s_app_settings = app_settings_defaults();
static std::atomic<bool> s_settings_save_pending{false};

static constexpr char MEASURE_NVS_NAMESPACE[] = "measure";
static constexpr char MEASURE_NVS_COUNTER_KEY[] = "session_id";
// Reserve IDs at boot while app_main is still running on an internal-RAM
// stack.  Runtime action tasks deliberately use PSRAM stacks to preserve
// Wi-Fi memory; those tasks must never write flash because PSRAM is
// inaccessible while the SPI flash cache is disabled.
static constexpr uint32_t MEASURE_SESSION_ID_RESERVATION = 1024;
static constexpr char ROOM_NVS_NAMESPACE[] = "room";
static constexpr char ROOM_NVS_COUNTER_KEY[] = "session_id";
static constexpr uint32_t ROOM_SESSION_ID_RESERVATION = 256;
struct DashboardState {
    bool runtime_started = false;
    bool startup_complete = false;
    uint8_t startup_pass_count = 0;
    uint8_t startup_warning_count = 0;
    uint8_t startup_fail_count = 0;
    DeviceCheckState check_lcd = DeviceCheckState::PENDING;
    DeviceCheckState check_i2c = DeviceCheckState::PENDING;
    DeviceCheckState check_input = DeviceCheckState::PENDING;
    DeviceCheckState check_sd = DeviceCheckState::PENDING;
    DeviceCheckState check_camera = DeviceCheckState::PENDING;
    DeviceCheckState check_laser_uart = DeviceCheckState::PENDING;
    bool sd_ready = false;
    bool bno_ready = false;
    bool laser_ready = false;
    int64_t t_us = 0;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float body_ax = 0.0f;
    float body_ay = 0.0f;
    float body_az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float body_gx = 0.0f;
    float body_gy = 0.0f;
    float body_gz = 0.0f;
    float qi = 0.0f;
    float qj = 0.0f;
    float qk = 0.0f;
    float qr = 1.0f;
    float body_qi = 0.0f;
    float body_qj = 0.0f;
    float body_qk = 0.0f;
    float body_qr = 1.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;
    uint8_t bno_status = 0;
    uint32_t bno_count = 0;
    uint32_t game_rv_count = 0;

    float path_x = 0.0f;
    float path_y = 0.0f;
    float path_z = 0.0f;
    float fusion_vx = 0.0f;
    float fusion_vy = 0.0f;
    float fusion_yaw = 0.0f;
    float fusion_pitch = 0.0f;
    float fusion_roll = 0.0f;
    float fusion_confidence = 0.0f;
    bool fusion_stationary = false;
    uint8_t measure_point_count = 0;
    uint32_t measure_session_id = 0;
    float point1_x = 0.0f;
    float point1_y = 0.0f;
    float point1_z = 0.0f;
    float point2_x = 0.0f;
    float point2_y = 0.0f;
    float point2_z = 0.0f;
    float point_distance_m = -1.0f;
    P2pStage p2p_stage = P2pStage::WAIT_A;
    bool p2p_a_valid = false;
    bool p2p_b_valid = false;
    int32_t p2p_distance_a_mm = 0;
    int32_t p2p_distance_b_mm = 0;
    int64_t p2p_timestamp_a_ms = 0;
    int64_t p2p_timestamp_b_ms = 0;
    float p2p_pitch_a_deg = 0.0f;
    float p2p_roll_a_deg = 0.0f;
    float p2p_yaw_a_deg = 0.0f;
    float p2p_pitch_b_deg = 0.0f;
    float p2p_roll_b_deg = 0.0f;
    float p2p_yaw_b_deg = 0.0f;
    float p2p_ax_m = 0.0f;
    float p2p_ay_m = 0.0f;
    float p2p_az_m = 0.0f;
    float p2p_bx_m = 0.0f;
    float p2p_by_m = 0.0f;
    float p2p_bz_m = 0.0f;
    float p2p_space_distance_m = -1.0f;
    float p2p_horizontal_distance_m = -1.0f;
    float p2p_height_diff_m = 0.0f;
    bool p2p_tripod_mode = true;
    uint8_t p2p_motion_risk = 0;
    float p2p_relative_angle_deg = 0.0f;
    float p2p_max_linear_accel_mps2 = 0.0f;
    float p2p_accel_deviation_sum = 0.0f;
    uint32_t p2p_accel_sample_count = 0;
    float p2p_max_gyro_dps = 0.0f;
    uint16_t p2p_imu_sync_error_ms = 0;
    int64_t p2p_motion_start_us = 0;
    FusionPose p2p_pose_a;
    FusionPose p2p_pose_b;
    bool p2p_pose_a_uses_game_rv = false;
    char p2p_image_a[96] = "";
    char p2p_image_b[96] = "";
    bool room_active = false;
    bool room_complete = false;
    uint32_t room_session_id = 0;
    uint16_t room_point_count = 0;
    float room_last_segment_m = 0.0f;
    float room_open_perimeter_m = 0.0f;
    float room_closure_m = 0.0f;
    float room_area_xy_m2 = 0.0f;
    float room_current_angle_deg = 0.0f;
    float room_rotation_deg = 0.0f;
    float room_pitch_deg = 0.0f;
    uint16_t room_valid_count = 0;
    uint16_t room_invalid_count = 0;
    bool room_pose_ready = false;
    uint8_t room_motion_risk = 0;
    float room_max_linear_accel_mps2 = 0.0f;
    bool room_scan_coverage_complete = false;
    char room_scan_file[48] = "";
    bool room_dxf_saved = false;
    char room_dxf_file[48] = "";
    uint16_t battery_adc_raw = 0;
    float battery_adc_voltage = 0.0f;
    float battery_voltage = 0.0f;
    uint8_t battery_percent = 0;
    bool battery_valid = false;

    int32_t laser_mm = -1;
    bool laser_busy = false;
    uint32_t laser_count = 0;
    char laser_raw[160] = "";

    bool key_measure = false;
    bool key_back = false;
    bool key_ok = false;
    uint8_t key_raw = 0xFF;
    uint32_t measure_press_count = 0;
    uint32_t back_press_count = 0;
    uint32_t ok_press_count = 0;
    uint32_t long_measure_press_count = 0;
    uint32_t long_back_press_count = 0;
    uint32_t long_ok_press_count = 0;

    uint32_t photo_count = 0;
    uint32_t last_saved_record_id = 0;
    uint16_t measurement_record_count = 0;
    uint8_t measurement_record_visible = 0;
    bool setting_auto_save_single = true;
    bool setting_photo_on_measure = false;
    uint8_t setting_distance_unit = 0;
    char last_photo[96] = "";
    char last_error[160] = "";
};

// 测量记录快照独立存放:避免把 8.7KB 数组塞进 DashboardState,
// 防止小栈任务(BNO 6KB/HTTP 12KB)拷贝整结构时依赖优化器 SROA 才不溢出。
static DeviceUiMeasurementRecord s_measurement_records[DEVICE_UI_RECORD_CAPACITY] = {};

static DashboardState s_dash;
static SemaphoreHandle_t s_dash_mutex = nullptr;
static SemaphoreHandle_t s_laser_mutex = nullptr;
static SemaphoreHandle_t s_sd_log_mutex = nullptr;
static TaskHandle_t s_key_task = nullptr;
static TaskHandle_t s_nvs_flush_task = nullptr;
static TaskHandle_t s_bno_task = nullptr;
static TaskHandle_t s_action_task = nullptr;
static TaskHandle_t s_battery_task = nullptr;
static SemaphoreHandle_t s_battery_adc_mutex = nullptr;
static adc_oneshot_unit_handle_t s_battery_adc = nullptr;
static adc_cali_handle_t s_battery_adc_cali = nullptr;
static adc_unit_t s_battery_adc_unit;
static adc_channel_t s_battery_adc_channel;
static std::atomic<bool> s_laser_measure_request{false};
static std::atomic<bool> s_laser_single_request{false};
static std::atomic<bool> s_single_distance_measure_request{false};
static std::atomic<bool> s_single_distance_save_request{false};
static std::atomic<bool> s_single_distance_cancel_request{false};
static std::atomic<bool> s_p2p_measure_request{false};
static std::atomic<bool> s_p2p_save_request{false};
static std::atomic<int> s_p2p_measure_target{-1};

struct TimedQuaternion {
    int64_t t_us = 0;
    float qi = 0.0f;
    float qj = 0.0f;
    float qk = 0.0f;
    float qr = 1.0f;
    uint8_t accuracy = 0;
};
static constexpr size_t P2P_IMU_HISTORY_CAPACITY = 64;
static std::array<TimedQuaternion, P2P_IMU_HISTORY_CAPACITY> s_game_rv_history = {};
static size_t s_game_rv_head = 0;
static size_t s_game_rv_count = 0;
static portMUX_TYPE s_game_rv_mux = portMUX_INITIALIZER_UNLOCKED;
// 单点测距基准偏移(毫米,浮点保留 0.1mm 实测精度;应用时四舍五入到整数):
//   前基准  = 激光直接读数 - 2mm            (offset = -2.0)
//   后基准  = 读数 + 126.5mm (offset = +126.5)
//   三脚架  = 读数 + (-FUSION_TRIPOD_AXIS_OFFSET_Y_M)；该值随 FUSION 标定更新
static constexpr float SINGLE_REAR_REFERENCE_OFFSET_MM = 126.5f;
static constexpr float SINGLE_FRONT_REFERENCE_OFFSET_MM = -2.0f;
// FUSION_TRIPOD_AXIS_OFFSET_Y_M is emitter -> tripod axis.  A negative Y value
// means the axis is behind the emitter, so the single-point distance correction
// along the laser ray is its opposite (for the current release calibration: +18.397 mm).
static constexpr float SINGLE_TRIPOD_REFERENCE_OFFSET_MM =
    -FUSION_TRIPOD_AXIS_OFFSET_Y_M * 1000.0f;
static std::atomic<bool> s_laser_active_requested{false};
static std::atomic<bool> s_laser_continuous_active{false};
static bool s_laser_startup_off_configured = false;
static bool s_laser_fast_rate_configured = false;
static int32_t s_laser_latest_mm = -1;
static int64_t s_laser_latest_us = 0;
static std::string s_laser_latest_raw;
static std::string s_laser_stream_line;
static std::vector<uint8_t> s_laser_stream_bytes;
static std::atomic<bool> s_photo_request{false};
static volatile bool s_web_toggle_request = false;
static std::atomic<bool> s_pc_unbind_request{false};  // UI 置位,action_task 异步执行解除绑定
static std::atomic<bool> s_pairing_manual_only{false};  // 解除绑定后仅接受手动绑定,禁用自动绑定
static volatile bool s_web_busy = false;
static volatile bool s_room_moving_mode = false;
static portMUX_TYPE s_measure_session_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_measure_session_next = 0;
static uint32_t s_measure_session_limit = 0;
static portMUX_TYPE s_room_session_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_room_session_next = 0;
static uint32_t s_room_session_limit = 0;

static constexpr size_t ROOM_MAX_POINTS = 64;
struct RoomSurveyState {
    bool active = false;
    bool complete = false;
    uint32_t session_id = 0;
    uint16_t count = 0;
    float x[ROOM_MAX_POINTS] = {};
    float y[ROOM_MAX_POINTS] = {};
    float z[ROOM_MAX_POINTS] = {};
    float last_segment_m = 0.0f;
    float open_perimeter_m = 0.0f;
    float closure_m = 0.0f;
    float area_xy_m2 = 0.0f;
    uint16_t scan_valid_count = 0;
    uint16_t scan_invalid_count = 0;
    int64_t scan_last_laser_us = 0;
    int64_t scan_last_saved_us = 0;
    float scan_start_azimuth_deg = 0.0f;
    float scan_last_azimuth_deg = 0.0f;
    float scan_unwrapped_deg = 0.0f;
    float scan_last_saved_angle_deg = 0.0f;
    int32_t scan_last_distance_mm = -1;
    float scan_start_roll_deg = 0.0f;
    float scan_start_pitch_deg = 0.0f;
    float scan_current_pitch_deg = 0.0f;
    float scan_max_linear_accel_mps2 = 0.0f;
    uint8_t scan_motion_risk = 0;
    bool scan_have_angle = false;
    bool scan_pose_ready = false;
    bool scan_coverage_complete = false;
    char scan_file[48] = "";
};
static RoomSurveyState s_room;
static FILE *s_room_scan_file = nullptr;
static bool p2p_pose_at(int64_t target_us, FusionPose *pose, uint16_t *error_ms);

struct OpenLog {
    const char *name;
    FILE *file;
    uint16_t pending;
};

static OpenLog s_open_logs[] = {
    {"events.csv", nullptr, 0}, {"bno086.csv", nullptr, 0},
    {"laser.csv", nullptr, 0}, {"measure_points.csv", nullptr, 0},
    {"measure_sessions.csv", nullptr, 0},
    {"room_points.csv", nullptr, 0}, {"room_sessions.csv", nullptr, 0},
    {"scan_sessions.csv", nullptr, 0},
    {"imu_calibration.csv", nullptr, 0},
};

static constexpr const char *kImuCalibrationLabels[] = {
    "HOME_Z_UP", "Z_DOWN", "X_UP_RIGHT_SIDE", "X_DOWN_LEFT_SIDE",
    "Y_UP_LASER", "Y_DOWN_LASER", "HOME_ROTATION_START", "YAW_LEFT_90",
    "HOME_AFTER_YAW", "PITCH_FRONT_UP_90", "HOME_AFTER_PITCH",
    "ROLL_RIGHT_DOWN_90", "HOME_FINAL",
};

static bool dashboard_toggle_web();

struct TestResult {
    std::string name;
    bool pass;
    std::string detail;
};

static std::vector<TestResult> s_summary;

static void json_escape_print(const char *s)
{
    putchar('"');
    for (; *s; ++s) {
        switch (*s) {
        case '\\': printf("\\\\"); break;
        case '"': printf("\\\""); break;
        case '\n': printf("\\n"); break;
        case '\r': printf("\\r"); break;
        case '\t': printf("\\t"); break;
        default:
            if (static_cast<unsigned char>(*s) < 0x20) {
                printf("\\u%04x", static_cast<unsigned char>(*s));
            } else {
                putchar(*s);
            }
        }
    }
    putchar('"');
}

static void log_line(const char *status, const char *test, const char *detail, bool record = true)
{
    printf("[%s] %s: %s\n", status, test, detail ? detail : "");
    printf("{\"test\":");
    json_escape_print(test);
    printf(",\"status\":");
    json_escape_print(status);
    printf(",\"data\":");
    json_escape_print(detail ? detail : "");
    printf("}\n");
    if (record && strcmp(status, "INFO") != 0) {
        s_summary.push_back({test, strcmp(status, "PASS") == 0, detail ? detail : ""});
    }
}

static void pass(const char *test, const std::string &detail) { log_line("PASS", test, detail.c_str()); }
static void fail(const char *test, const std::string &detail) { log_line("FAIL", test, detail.c_str()); }
static void info(const char *test, const std::string &detail) { log_line("INFO", test, detail.c_str(), false); }

static void dashboard_lock()
{
    if (s_dash_mutex) {
        xSemaphoreTake(s_dash_mutex, portMAX_DELAY);
    }
}

static void dashboard_unlock()
{
    if (s_dash_mutex) {
        xSemaphoreGive(s_dash_mutex);
    }
}

static void dashboard_log_event(const char *type, const std::string &detail);

static void dashboard_set_error(const std::string &msg)
{
    dashboard_lock();
    snprintf(s_dash.last_error, sizeof(s_dash.last_error), "%s", msg.c_str());
    dashboard_unlock();
    ESP_LOGE("product", "%s", msg.c_str());
    dashboard_log_event("error", msg);
}

static bool dashboard_log_csv(const char *filename, const char *line)
{
    if (!s_sd_mounted || !filename || !line) {
        return false;
    }
    if (!s_sd_log_mutex || xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return false;
    }
    OpenLog *log = nullptr;
    for (auto &candidate : s_open_logs) {
        if (strcmp(candidate.name, filename) == 0) { log = &candidate; break; }
    }
    if (log && !log->file) {
        std::string path = std::string(SD_MOUNT_POINT) + "/" + filename;
        log->file = fopen(path.c_str(), "a");
        if (log->file) setvbuf(log->file, nullptr, _IOFBF, 4096);
    }
    bool written = false;
    if (log && log->file) {
        written = fputs(line, log->file) >= 0;
        if (++log->pending >= 10 || strcmp(filename, "events.csv") == 0 ||
            strcmp(filename, "laser.csv") == 0 || strcmp(filename, "measure_points.csv") == 0 ||
            strcmp(filename, "measure_sessions.csv") == 0 ||
            strcmp(filename, "room_points.csv") == 0 || strcmp(filename, "room_sessions.csv") == 0 ||
            strcmp(filename, "imu_calibration.csv") == 0) {
            if (fflush(log->file) != 0) written = false;
            log->pending = 0;
        }
    }
    xSemaphoreGive(s_sd_log_mutex);
    return written;
}

static void dashboard_log_event(const char *type, const std::string &detail)
{
    char line[320];
    snprintf(line, sizeof(line), "%lld,%s,%s\n",
             static_cast<long long>(esp_timer_get_time()),
             type ? type : "event", detail.c_str());
    dashboard_log_csv("events.csv", line);
}

static void publish_app_settings(const AppSettings &settings)
{
    dashboard_lock();
    s_dash.setting_auto_save_single = settings.auto_save_single;
    s_dash.setting_photo_on_measure = settings.photo_on_measure;
    s_dash.setting_distance_unit = static_cast<uint8_t>(settings.distance_unit);
    dashboard_unlock();
}

static void load_app_settings()
{
    AppSettings loaded = app_settings_defaults();
    esp_err_t err = app_settings_load(&loaded);
    if (err != ESP_OK) {
        dashboard_set_error("settings load failed: " + std::string(esp_err_to_name(err)));
        loaded = app_settings_defaults();
    }
    // Single-distance results are always reviewed before a manual save.
    // Migrate older NVS values that could skip the frozen-result screen.
    // NOTE: distance_unit is a user setting (mm/cm/m) and must NOT be reset here;
    // only the removed auto-save/photo flags are migrated to their fixed values.
    const bool migrated = loaded.auto_save_single || !loaded.photo_on_measure;
    loaded.auto_save_single = false;
    loaded.photo_on_measure = true;
    if (migrated) app_settings_save(loaded);
    dashboard_lock();
    s_app_settings = loaded;
    dashboard_unlock();
    publish_app_settings(loaded);
}

static void process_pending_settings_save()
{
    if (!s_settings_save_pending.exchange(false, std::memory_order_acq_rel)) return;
    AppSettings settings;
    dashboard_lock();
    settings = s_app_settings;
    dashboard_unlock();
    esp_err_t err = app_settings_save(settings);
    if (err == ESP_OK) {
        dashboard_log_event("settings", "saved");
    } else {
        dashboard_set_error("settings save failed: " + std::string(esp_err_to_name(err)));
    }
}

// NVS 待保存泵:PC 绑定/设置/时间同步的待决写由本任务执行,不再依赖串口 fgets 循环。
// NVS 提交会短暂关闭 Flash cache,因此本任务必须使用内部 RAM 栈。
static void nvs_flush_task(void *)
{
    while (true) {
        process_pending_pc_binding_save();
        process_pending_settings_save();
        process_pending_time_sync_save();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void publish_measurement_records()
{
    MeasurementRecord records[DEVICE_UI_RECORD_CAPACITY] = {};
    size_t total = 0;
    const size_t count = s_measurement_store.snapshot_newest(
        records, DEVICE_UI_RECORD_CAPACITY, &total);
    dashboard_lock();
    s_dash.measurement_record_count = static_cast<uint16_t>(std::min<size_t>(total, UINT16_MAX));
    s_dash.measurement_record_visible = static_cast<uint8_t>(count);
    memset(s_measurement_records, 0, sizeof(s_measurement_records));
    for (size_t i = 0; i < count; ++i) {
        s_measurement_records[i].id = records[i].id;
        s_measurement_records[i].t_us = records[i].t_us;
        s_measurement_records[i].distance_mm = records[i].distance_mm;
        s_measurement_records[i].reference = records[i].reference;
        s_measurement_records[i].pitch_deg = records[i].pitch_deg;
        s_measurement_records[i].roll_deg = records[i].roll_deg;
        s_measurement_records[i].yaw_deg = records[i].yaw_deg;
        s_measurement_records[i].camera_frame_saved = records[i].camera_frame_saved;
        snprintf(s_measurement_records[i].image_path,
                 sizeof(s_measurement_records[i].image_path), "%s", records[i].image_path);
        s_measurement_records[i].quality = records[i].quality;
        s_measurement_records[i].measure_time_ms = records[i].measure_time_ms;
        s_measurement_records[i].raw_error_code = records[i].raw_error_code;
        s_measurement_records[i].type = static_cast<DeviceUiMeasurementRecord::Type>(records[i].type);
        s_measurement_records[i].floorplan_number = 0;
        s_measurement_records[i].distance_a_mm = records[i].distance_a_mm;
        s_measurement_records[i].distance_b_mm = records[i].distance_b_mm;
        s_measurement_records[i].horizontal_mm = records[i].horizontal_mm;
        s_measurement_records[i].height_diff_mm = records[i].height_diff_mm;
        s_measurement_records[i].timestamp_a_ms = records[i].timestamp_a_ms;
        s_measurement_records[i].timestamp_b_ms = records[i].timestamp_b_ms;
        snprintf(s_measurement_records[i].image_path_b,
                 sizeof(s_measurement_records[i].image_path_b), "%s", records[i].image_path_b);
    }
    dashboard_unlock();
}

static void finalize_startup_checks()
{
    dashboard_lock();
    const DeviceCheckState checks[] = {
        s_dash.check_lcd, s_dash.check_i2c, s_dash.check_input,
        s_dash.check_sd, s_dash.check_camera, s_dash.check_laser_uart,
    };
    s_dash.startup_pass_count = 0;
    s_dash.startup_warning_count = 0;
    s_dash.startup_fail_count = 0;
    for (DeviceCheckState check : checks) {
        if (check == DeviceCheckState::PASS) ++s_dash.startup_pass_count;
        else if (check == DeviceCheckState::WARNING) ++s_dash.startup_warning_count;
        else if (check == DeviceCheckState::FAIL) ++s_dash.startup_fail_count;
    }
    s_dash.startup_complete = true;
    const uint8_t passed = s_dash.startup_pass_count;
    const uint8_t warnings = s_dash.startup_warning_count;
    const uint8_t failed = s_dash.startup_fail_count;
    dashboard_unlock();
    char detail[96];
    snprintf(detail, sizeof(detail), "complete pass=%u warning=%u fail=%u", passed, warnings, failed);
    dashboard_log_event("startup", detail);
    ESP_LOGI("startup", "%s", detail);
}

static std::string esp_err_str(esp_err_t err)
{
    return std::string(esp_err_to_name(err)) + " (0x" + std::to_string(static_cast<int>(err)) + ")";
}

static std::string hex_addr(uint8_t addr)
{
    char b[8];
    snprintf(b, sizeof(b), "0x%02X", addr);
    return b;
}

static esp_err_t i2c_init_once()
{
    if (s_i2c_ready) {
        return ESP_OK;
    }
    if (!s_i2c_init_mutex) {
        s_i2c_init_mutex = xSemaphoreCreateMutex();
        if (!s_i2c_init_mutex) return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_i2c_init_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_i2c_ready) {
        xSemaphoreGive(s_i2c_init_mutex);
        return ESP_OK;
    }
    if (!s_i2c_mutex) {
        s_i2c_mutex = xSemaphoreCreateMutex();
    }
    i2c_master_bus_config_t conf = {};
    conf.i2c_port = I2C_PORT;
    conf.sda_io_num = PIN_I2C_SDA;
    conf.scl_io_num = PIN_I2C_SCL;
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.glitch_ignore_cnt = 7;
    conf.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t new_bus = nullptr;
    esp_err_t err = i2c_new_master_bus(&conf, &new_bus);
    if (err == ESP_OK) {
        s_i2c_bus = new_bus;
        s_i2c_ready = true;
    }
    xSemaphoreGive(s_i2c_init_mutex);
    return err;
}

static bool i2c_take_bus(int timeout_ms = I2C_TIMEOUT_MS)
{
    if (!s_i2c_mutex) {
        s_i2c_mutex = xSemaphoreCreateMutex();
    }
    if (!s_i2c_mutex) {
        return false;
    }
    return xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void i2c_give_bus()
{
    if (s_i2c_mutex) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

static void i2c_release_bus()
{
    bool locked = i2c_take_bus(500);
    if (!locked) {
        info("i2c", "release skipped: bus busy");
        return;
    }
    if (s_i2c_ready && s_i2c_bus) {
        esp_err_t err = i2c_del_master_bus(s_i2c_bus);
        if (err != ESP_OK) {
            info("i2c", "release failed: " + esp_err_str(err));
        }
    }
    s_i2c_ready = false;
    s_i2c_bus = nullptr;
    i2c_give_bus();
}

static void i2c_gpio_bus_recover(const char *tag)
{
    gpio_reset_pin(PIN_I2C_SDA);
    gpio_reset_pin(PIN_I2C_SCL);

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_I2C_SDA) | (1ULL << PIN_I2C_SCL);
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);

    gpio_set_level(PIN_I2C_SDA, 1);
    gpio_set_level(PIN_I2C_SCL, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    // A CPU-only reset can interrupt a read while the slave is driving SDA.
    // Clock up to nine remaining bits, then generate a STOP condition before
    // installing the ESP-IDF master driver. The I2C peripherals remain powered
    // when CH340 DTR/RTS resets only the ESP32-S3.
    for (int i = 0; i < 9; ++i) {
        gpio_set_level(PIN_I2C_SCL, 0);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_I2C_SCL, 1);
        esp_rom_delay_us(10);
    }
    gpio_set_level(PIN_I2C_SCL, 0);
    gpio_set_level(PIN_I2C_SDA, 0);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_I2C_SDA, 1);
    esp_rom_delay_us(10);

    int sda = gpio_get_level(PIN_I2C_SDA);
    int scl = gpio_get_level(PIN_I2C_SCL);
    info(tag ? tag : "i2c", "GPIO bus release SDA=" + std::to_string(sda) + " SCL=" + std::to_string(scl));
}

struct I2cLineDiag {
    int idle_sda = -1;
    int idle_scl = -1;
    int scl_low_sda = -1;
    int scl_low_scl = -1;
    int scl_release_sda = -1;
    int scl_release_scl = -1;
    int sda_low_sda = -1;
    int sda_low_scl = -1;
    int sda_release_sda = -1;
    int sda_release_scl = -1;
    int cam_rst_low_sda = -1;
    int cam_rst_low_scl = -1;
    int cam_rst_high_sda = -1;
    int cam_rst_high_scl = -1;
    bool camera_reset_sampled = false;
};

static I2cLineDiag i2c_runtime_line_diag()
{
    I2cLineDiag d;
    if (i2c_init_once() != ESP_OK || !i2c_take_bus(500)) {
        return d;
    }
    // Holding the application bus mutex prevents sampling SDA/SCL halfway
    // through a valid BNO, touch, or PCA transfer.
    d.idle_sda = gpio_get_level(PIN_I2C_SDA);
    d.idle_scl = gpio_get_level(PIN_I2C_SCL);
    i2c_give_bus();
    return d;
}

static void i2c_gpio_prepare_open_drain()
{
    gpio_reset_pin(PIN_I2C_SDA);
    gpio_reset_pin(PIN_I2C_SCL);

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_I2C_SDA) | (1ULL << PIN_I2C_SCL);
    cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);
    gpio_set_level(PIN_I2C_SDA, 1);
    gpio_set_level(PIN_I2C_SCL, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static I2cLineDiag i2c_line_diag(bool sample_camera_reset)
{
    i2c_release_bus();
    SCCB_Deinit();
    i2c_gpio_prepare_open_drain();

    I2cLineDiag d;
    d.idle_sda = gpio_get_level(PIN_I2C_SDA);
    d.idle_scl = gpio_get_level(PIN_I2C_SCL);

    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(20);
    d.scl_low_sda = gpio_get_level(PIN_I2C_SDA);
    d.scl_low_scl = gpio_get_level(PIN_I2C_SCL);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(50);
    d.scl_release_sda = gpio_get_level(PIN_I2C_SDA);
    d.scl_release_scl = gpio_get_level(PIN_I2C_SCL);

    gpio_set_level(PIN_I2C_SDA, 0);
    esp_rom_delay_us(20);
    d.sda_low_sda = gpio_get_level(PIN_I2C_SDA);
    d.sda_low_scl = gpio_get_level(PIN_I2C_SCL);
    gpio_set_level(PIN_I2C_SDA, 1);
    esp_rom_delay_us(50);
    d.sda_release_sda = gpio_get_level(PIN_I2C_SDA);
    d.sda_release_scl = gpio_get_level(PIN_I2C_SCL);

    if (sample_camera_reset && !s_camera_http_ready) {
        gpio_config_t rst_cfg = {};
        rst_cfg.pin_bit_mask = (1ULL << PIN_CAM_RST);
        rst_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&rst_cfg);
        gpio_set_level(PIN_CAM_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(30));
        i2c_gpio_prepare_open_drain();
        d.cam_rst_low_sda = gpio_get_level(PIN_I2C_SDA);
        d.cam_rst_low_scl = gpio_get_level(PIN_I2C_SCL);
        gpio_set_level(PIN_CAM_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
        i2c_gpio_prepare_open_drain();
        d.cam_rst_high_sda = gpio_get_level(PIN_I2C_SDA);
        d.cam_rst_high_scl = gpio_get_level(PIN_I2C_SCL);
        d.camera_reset_sampled = true;
    }
    return d;
}

static std::string i2c_line_verdict(const I2cLineDiag &d)
{
    if (d.idle_sda == 1 && d.idle_scl == 1) {
        return "idle high: bus electrical idle looks OK";
    }
    if (d.idle_sda == 0 && d.idle_scl == 0) {
        return "SDA and SCL both held low: likely short/wrong FPC pinout/unpowered module/level shifter stuck";
    }
    if (d.idle_sda == 0) {
        return "SDA held low: a device may be stuck mid-transfer or SDA is shorted";
    }
    return "SCL held low: a device may be clock-stretching/stuck or SCL is shorted";
}

static std::string i2c_line_detail(const I2cLineDiag &d)
{
    char b[320];
    snprintf(b, sizeof(b),
             "idle=%d/%d scl_low=%d/%d scl_rel=%d/%d sda_low=%d/%d sda_rel=%d/%d cam_rst_low=%d/%d cam_rst_high=%d/%d %s",
             d.idle_sda, d.idle_scl,
             d.scl_low_sda, d.scl_low_scl,
             d.scl_release_sda, d.scl_release_scl,
             d.sda_low_sda, d.sda_low_scl,
             d.sda_release_sda, d.sda_release_scl,
             d.cam_rst_low_sda, d.cam_rst_low_scl,
             d.cam_rst_high_sda, d.cam_rst_high_scl,
             i2c_line_verdict(d).c_str());
    return b;
}

static esp_err_t camera_use_shared_i2c(camera_config_t &config)
{
    // esp32-camera supports attaching its SCCB device to an I2C master bus
    // that the application already owns.  Keeping one driver installed avoids
    // the old hand-off window where the camera and runtime devices could each
    // try to install a driver on the same GPIOs/controller.
    ESP_RETURN_ON_ERROR(i2c_init_once(), "camera", "shared I2C init for camera");
    config.pin_sccb_sda = -1;
    config.pin_sccb_scl = -1;
    config.sccb_i2c_port = I2C_PORT;
    info("camera", "SCCB uses shared I2C port " + std::to_string(static_cast<int>(I2C_PORT)));
    return ESP_OK;
}

static camera_config_t camera_jpeg_config(framesize_t frame_size, int quality,
                                          int fb_count, camera_grab_mode_t grab_mode)
{
    camera_config_t config = {};
    config.pin_pwdn = -1;
    config.pin_reset = PIN_CAM_RST;
    config.pin_xclk = PIN_CAM_XCLK;
    config.pin_d7 = PIN_CAM_D7;
    config.pin_d6 = PIN_CAM_D6;
    config.pin_d5 = PIN_CAM_D5;
    config.pin_d4 = PIN_CAM_D4;
    config.pin_d3 = PIN_CAM_D3;
    config.pin_d2 = PIN_CAM_D2;
    config.pin_d1 = PIN_CAM_D1;
    config.pin_d0 = PIN_CAM_D0;
    config.pin_vsync = PIN_CAM_VSYNC;
    config.pin_href = PIN_CAM_HREF;
    config.pin_pclk = PIN_CAM_PCLK;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_1;
    config.ledc_channel = LEDC_CHANNEL_1;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = frame_size;
    config.jpeg_quality = quality;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = grab_mode;
    return config;
}

static bool i2c_probe(uint8_t addr)
{
    if (i2c_init_once() != ESP_OK) {
        return false;
    }
    if (!i2c_take_bus(I2C_PROBE_MUTEX_TIMEOUT_MS)) {
        return false;
    }
    bool ok = i2c_master_probe(s_i2c_bus, addr, I2C_TIMEOUT_MS) == ESP_OK;
    i2c_give_bus();
    return ok;
}

static esp_err_t i2c_add_temp_device(uint8_t addr, i2c_master_dev_handle_t *dev)
{
    if (i2c_init_once() != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = I2C_FREQ_HZ;
    return i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, dev);
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

static esp_err_t i2c_write_raw(uint8_t addr, const uint8_t *data, size_t len, int timeout_ms = I2C_TIMEOUT_MS)
{
    if (!i2c_take_bus(timeout_ms + I2C_MUTEX_TIMEOUT_MS)) {
        return ESP_ERR_TIMEOUT;
    }
    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t err = i2c_add_temp_device(addr, &dev);
    if (err == ESP_OK) {
        err = i2c_master_transmit(dev, data, len, timeout_ms);
        i2c_master_bus_rm_device(dev);
    }
    i2c_give_bus();
    return err;
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_write_raw(addr, data, sizeof(data));
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return i2c_read_regs(addr, reg, value, 1);
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (!i2c_take_bus(I2C_MUTEX_TIMEOUT_MS)) {
        return ESP_ERR_TIMEOUT;
    }
    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t err = i2c_add_temp_device(addr, &dev);
    if (err == ESP_OK) {
        err = i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
        i2c_master_bus_rm_device(dev);
    }
    i2c_give_bus();
    return err;
}

static std::vector<uint8_t> i2c_scan_addresses()
{
    std::vector<uint8_t> found;
    if (i2c_init_once() != ESP_OK || !i2c_take_bus(2000)) {
        return found;
    }
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(s_i2c_bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            found.push_back(addr);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    i2c_give_bus();
    return found;
}

static bool has_addr(const std::vector<uint8_t> &list, uint8_t addr)
{
    return std::find(list.begin(), list.end(), addr) != list.end();
}

static bool bno_select_i2c_addr()
{
    const uint8_t candidates[] = {I2C_ADDR_BNO086, I2C_ADDR_BNO086_ALT};
    for (uint8_t addr : candidates) {
        if (i2c_probe(addr)) {
            s_bno_addr = addr;
            return true;
        }
    }
    return false;
}

static std::string join_addrs(const std::vector<uint8_t> &list)
{
    std::string out;
    for (uint8_t a : list) {
        if (!out.empty()) {
            out += ",";
        }
        out += hex_addr(a);
    }
    return out.empty() ? "(none)" : out;
}

class Pca9557 {
public:
    esp_err_t begin()
    {
        ESP_RETURN_ON_ERROR(i2c_init_once(), TAG, "i2c init failed");
        // The revised board uses IO0..2 for the three keys and leaves IO3..7
        // unconnected.  BACK and OK intentionally retain the product's
        // original logical swap (IO2=BACK, IO0=OK).  Keep all ports as inputs;
        // the touch controller has no PCA9557 reset connection on this revision.
        constexpr uint8_t cfg = 0xFF;
        uint8_t out = 0xFF;
        ESP_RETURN_ON_ERROR(i2c_write_reg(PCA9557_ADDR, REG_OUTPUT, out), TAG, "pca output failed");
        ESP_RETURN_ON_ERROR(i2c_write_reg(PCA9557_ADDR, REG_POLARITY, 0x00), TAG, "pca polarity failed");
        ESP_RETURN_ON_ERROR(i2c_write_reg(PCA9557_ADDR, REG_CONFIG, cfg), TAG, "pca config failed");
        output_cache_ = out;
        uint8_t touch_id = 0;
        esp_err_t touch_err = i2c_read_reg(I2C_ADDR_TOUCH, 0xA7, &touch_id);
        if (touch_err == ESP_OK) {
            ESP_LOGI(TAG, "touch controller 0x15 chip_id=0x%02X", touch_id);
        } else {
            ESP_LOGW(TAG, "touch controller 0x15 did not respond: %s",
                     esp_err_to_name(touch_err));
        }
        return ESP_OK;
    }

    esp_err_t readInput(uint8_t *value) { return i2c_read_reg(PCA9557_ADDR, REG_INPUT, value); }

    esp_err_t writePin(uint8_t pin, bool level)
    {
        if (level) {
            output_cache_ |= (1u << pin);
        } else {
            output_cache_ &= ~(1u << pin);
        }
        return i2c_write_reg(PCA9557_ADDR, REG_OUTPUT, output_cache_);
    }

private:
    static constexpr const char *TAG = "pca9557";
    static constexpr uint8_t REG_INPUT = 0x00;
    static constexpr uint8_t REG_OUTPUT = 0x01;
    static constexpr uint8_t REG_POLARITY = 0x02;
    static constexpr uint8_t REG_CONFIG = 0x03;
    uint8_t output_cache_ = 0xFF;
};

static Pca9557 s_pca;

static void print_help()
{
    printf("\nCommands:\n");
    printf("  help\n");
    printf("  pinmap\n");
    printf("  test_power_hint\n");
    printf("  diag_i2c_lines\n");
    printf("  recover_i2c\n");
    printf("  scan_i2c\n");
    printf("  test_pca9557\n");
    printf("  test_keys\n");
    printf("  test_lcd\n");
    printf("  test_touch\n");
    printf("  test_sd\n");
    printf("  test_sd_speed\n");
    printf("  test_camera\n");
    printf("  capture_dataset\n");
    printf("  start_camera_ap\n");
    printf("  stop_camera_ap\n");
    printf("  stream_camera_uart [frames]\n");
    printf("  test_bno086\n");
    printf("  measure_laser\n");
    printf("  test_laser_once\n");
    printf("  reset_pose\n");
    printf("  reset_points\n");
    printf("  test_laser_cont\n");
    printf("  test_bat_adc\n");
    printf("  test_wifi\n");
    printf("  test_all\n");
    printf("  runtime_status\n");
    printf("  stop_laser\n\n");
    printf("  ui_camera\n");
    printf("  ui_single\n");
    printf("  ui_p2p\n");
    printf("  ui_menu\n\n");
    printf("  calibrate_imu\n");
    printf("  imu_cal_clear\n");
    printf("  imu_cal_dump\n");
    printf("  room_dump\n");
    printf("  room_export_last\n\n");
    printf("  pc_status\n");
    printf("  pc_unbind\n\n");
}

static void cmd_pinmap()
{
    printf("\nPinmap:\n");
    printf("I2C SDA=%d SCL=%d, BNO086_INT=%d, BAT_ADC=%d\n", PIN_I2C_SDA, PIN_I2C_SCL, PIN_BNO086_INT, PIN_BAT_ADC);
    printf("SD D0=%d CLK=%d CMD=%d\n", PIN_SD_D0, PIN_SD_CLK, PIN_SD_CMD);
    printf("LCD NV3030B CS=%d SCLK=%d MOSI/D0=%d D1(unused)=%d, D2/D3 disconnected\n",
           PIN_LCD_CS, PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_D1);
    printf("Touch polling SDA=%d SCL=%d addr=0x%02X (backboard auto-reset, no INT)\n",
           PIN_I2C_SDA, PIN_I2C_SCL, I2C_ADDR_TOUCH);
    printf("Laser RX=%d TX=%d baud=%d\n", PIN_LASER_RX, PIN_LASER_TX, LASER_BAUD);
    printf("Camera XCLK=%d PCLK=%d VSYNC=%d HREF=%d RST=%d D0..D7=%d,%d,%d,%d,%d,%d,%d,%d\n",
           PIN_CAM_XCLK, PIN_CAM_PCLK, PIN_CAM_VSYNC, PIN_CAM_HREF, PIN_CAM_RST,
           PIN_CAM_D0, PIN_CAM_D1, PIN_CAM_D2, PIN_CAM_D3, PIN_CAM_D4, PIN_CAM_D5, PIN_CAM_D6, PIN_CAM_D7);
    printf("Debug UART TX=%d RX=%d, BOOT=%d\n\n", PIN_DEBUG_TXD, PIN_DEBUG_RXD, PIN_BOOT);
    info("pinmap", "printed board pin map");
}

static void cmd_power_hint()
{
    printf("\nManual power checks before plugging modules:\n");
    printf("  5V_SYS: 4.8-5.2 V\n");
    printf("  VDD_3V3 / 3V3_SYS: 3.20-3.40 V\n");
    printf("  DOVDD_2V8 / AVDD_2V8: 2.70-2.90 V\n");
    printf("  DVDD_1V5: 1.43-1.57 V\n");
    printf("  First power-up: 5 V with 200 mA current limit, no obvious heating.\n\n");
    info("test_power_hint", "manual voltage ranges printed");
}

static bool cmd_diag_i2c_lines()
{
    I2cLineDiag d;
    if (s_camera_http_ready) {
        // The camera is a registered device on the shared bus.  Do not
        // uninstall/bit-bang that bus while esp_camera still owns the sensor
        // handle; a passive sample is sufficient for the runtime diagnostic.
        d = i2c_runtime_line_diag();
    } else {
        d = i2c_line_diag(true);
    }
    bool ok = d.idle_sda == 1 && d.idle_scl == 1;
    ok ? pass("diag_i2c_lines", i2c_line_detail(d))
       : fail("diag_i2c_lines", i2c_line_detail(d));
    return ok;
}

static bool cmd_recover_i2c()
{
    if (s_camera_http_ready) {
        fail("recover_i2c", "refused while camera is active on the shared I2C bus; reboot performs safe boot recovery");
        return false;
    }
    i2c_release_bus();
    SCCB_Deinit();
    i2c_gpio_bus_recover("recover_i2c");
    I2cLineDiag d = i2c_line_diag(false);
    bool ok = d.idle_sda == 1 && d.idle_scl == 1;
    ok ? pass("recover_i2c", i2c_line_detail(d))
       : fail("recover_i2c", i2c_line_detail(d));
    return ok;
}

static bool cmd_scan_i2c()
{
    I2cLineDiag d;
    if (s_camera_http_ready) {
        d = i2c_runtime_line_diag();
    } else {
        d = i2c_line_diag(false);
    }
    int sda_level = d.idle_sda;
    int scl_level = d.idle_scl;

    if (sda_level == 0 || scl_level == 0) {
        std::string detail = i2c_line_detail(d);
        fail("scan_i2c", detail);
        return false;
    }

    auto found = i2c_scan_addresses();
    std::string detail = "SDA=" + std::to_string(sda_level) + " SCL=" + std::to_string(scl_level) +
                         " found=" + join_addrs(found);
    if (!has_addr(found, PCA9557_ADDR)) detail += " missing:PCA9557";
    if (!has_addr(found, I2C_ADDR_BNO086) && !has_addr(found, I2C_ADDR_BNO086_ALT)) detail += " missing:BNO086";
    if (!has_addr(found, I2C_ADDR_TOUCH)) detail += " missing:TOUCH?";
    if (!has_addr(found, I2C_ADDR_OV5640)) detail += " missing:OV5640?";
    bool ok = !found.empty() && has_addr(found, PCA9557_ADDR);
    ok ? pass("scan_i2c", detail) : fail("scan_i2c", detail);
    return ok;
}

static bool cmd_test_pca9557()
{
    esp_err_t err = s_pca.begin();
    if (err != ESP_OK) {
        fail("test_pca9557", "init failed: " + esp_err_str(err));
        return false;
    }
    uint8_t input = 0;
    err = s_pca.readInput(&input);
    if (err != ESP_OK) {
        fail("test_pca9557", "input read failed: " + esp_err_str(err));
        return false;
    }
    char b[96];
    snprintf(b, sizeof(b), "addr=0x%02X input=0x%02X all ports input; IO0/1/2 are keys", PCA9557_ADDR, input);
    pass("test_pca9557", b);
    return true;
}

static bool cmd_test_keys()
{
    if (s_pca.begin() != ESP_OK) {
        fail("test_keys", "PCA9557 not ready");
        return false;
    }
    printf("Press/release MEASURE BACK OK during the next 10 seconds.\n");
    bool seen_press[3] = {};
    bool seen_release[3] = {};
    const uint8_t pins[3] = {PCA_IO_KEY_MEASURE, PCA_IO_KEY_BACK, PCA_IO_KEY_OK};
    const char *names[3] = {"MEASURE", "BACK", "OK"};
    int64_t end = esp_timer_get_time() + 10LL * 1000 * 1000;
    while (esp_timer_get_time() < end) {
        uint8_t in = 0;
        if (s_pca.readInput(&in) == ESP_OK) {
            std::string line;
            for (int i = 0; i < 3; ++i) {
                bool level = in & (1u << pins[i]);
                bool pressed = KEY_ACTIVE_LOW ? !level : level;
                seen_press[i] |= pressed;
                seen_release[i] |= !pressed;
                line += names[i];
                line += pressed ? "=DOWN " : "=UP ";
            }
            info("test_keys", line);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    bool ok = true;
    std::string detail;
    for (int i = 0; i < 3; ++i) {
        ok &= seen_press[i] && seen_release[i];
        detail += names[i];
        detail += seen_press[i] ? ":pressed " : ":no_press ";
    }
    ok ? pass("test_keys", detail) : fail("test_keys", detail + "check key wiring or active level");
    return ok;
}

#if 0  // Legacy GC9307/PCA9557 LCD implementation; replaced by nv3030b_lcd.cpp.
static esp_err_t ledc_backlight_init()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), "lcd", "ledc timer");

    ledc_channel_config_t ch = {};
    ch.gpio_num = PIN_LCD_BL;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel = LEDC_CHANNEL_0;
    ch.intr_type = LEDC_INTR_DISABLE;
    ch.timer_sel = LEDC_TIMER_0;
    ch.duty = 0;
    ch.hpoint = 0;
    return ledc_channel_config(&ch);
}

static void lcd_set_backlight(uint32_t percent)
{
    percent = std::min<uint32_t>(percent, 100);
    uint32_t duty = (1023 * percent) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static esp_err_t lcd_spi_init_once()
{
    if (s_lcd_spi) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(s_pca.begin(), "lcd", "pca begin");
    ESP_RETURN_ON_ERROR(ledc_backlight_init(), "lcd", "backlight init");

    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_LCD_MOSI;
    bus.miso_io_num = GPIO_NUM_NC;
    bus.sclk_io_num = PIN_LCD_SCLK;
    bus.quadwp_io_num = GPIO_NUM_NC;
    bus.quadhd_io_num = GPIO_NUM_NC;
    bus.max_transfer_sz = LCD_WIDTH * 80 * 2 + 16;
    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = 1 * 1000 * 1000;
    dev.mode = 0;
    dev.spics_io_num = GPIO_NUM_NC;
    dev.queue_size = 1;
    return spi_bus_add_device(LCD_SPI_HOST, &dev, &s_lcd_spi);
}

static esp_err_t lcd_spi_write(bool dc, const uint8_t *data, size_t len)
{
    if (!s_lcd_spi || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    gpio_set_level(PIN_LCD_DC, dc ? 1 : 0);
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_CS, false), "lcd", "cs low");
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    esp_err_t err = spi_device_transmit(s_lcd_spi, &t);
    esp_err_t cs_err = s_pca.writePin(PCA_IO_LCD_CS, true);
    if (err != ESP_OK) {
        return err;
    }
    return cs_err;
}

static esp_err_t lcd_cmd(uint8_t cmd)
{
    return lcd_spi_write(false, &cmd, 1);
}

static esp_err_t lcd_data(const uint8_t *data, size_t len)
{
    return lcd_spi_write(true, data, len);
}

static esp_err_t lcd_cmd_data(uint8_t cmd, const std::initializer_list<uint8_t> &data)
{
    ESP_RETURN_ON_ERROR(lcd_cmd(cmd), "lcd", "cmd");
    if (data.size() > 0) {
        ESP_RETURN_ON_ERROR(lcd_data(data.begin(), data.size()), "lcd", "data");
    }
    return ESP_OK;
}

static esp_err_t lcd_init_gc9307()
{
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_LCD_DC);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);

    s_pca.writePin(PCA_IO_LCD_CS, true);
    s_pca.writePin(PCA_IO_LCD_RST, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    s_pca.writePin(PCA_IO_LCD_RST, true);
    vTaskDelay(pdMS_TO_TICKS(120));
    s_pca.writePin(PCA_IO_LCD_CS, false);

    ESP_RETURN_ON_ERROR(lcd_cmd(0x01), "lcd", "swreset");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(lcd_cmd(0x11), "lcd", "sleep out");
    vTaskDelay(pdMS_TO_TICKS(120));

    // YH-201BGC285C0, 240x296, GC9307, 4-line SPI. Values copied from the
    // vendor reference project's USE_SPI_LCD_201_GC9307 init table.
    ESP_RETURN_ON_ERROR(lcd_cmd(0xFE), "lcd", "gc9307 unlock");
    ESP_RETURN_ON_ERROR(lcd_cmd(0xEF), "lcd", "gc9307 unlock2");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x36, {0x48}), "lcd", "madctl");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x3A, {0x05}), "lcd", "rgb565");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x84, {0x40}), "lcd", "set 84");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x86, {0x98}), "lcd", "set 86");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x89, {0x13}), "lcd", "set 89");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x8B, {0x80}), "lcd", "set 8b");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x8D, {0x33}), "lcd", "set 8d");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x8E, {0x0F}), "lcd", "set 8e");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xB6, {0x00, 0x00, 0x24}), "lcd", "set b6");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xE8, {0x13, 0x00}), "lcd", "set e8");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xEC, {0x33, 0x00, 0xF0}), "lcd", "set ec");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xFF, {0x62}), "lcd", "set ff");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x99, {0x3E}), "lcd", "set 99");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x9D, {0x4B}), "lcd", "set 9d");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x98, {0x3E}), "lcd", "set 98");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0x9C, {0x4B}), "lcd", "set 9c");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xC3, {0x2A}), "lcd", "vreg1a");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xC4, {0x14}), "lcd", "vreg1b");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xC9, {0x34}), "lcd", "vreg2a");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xF0, {0x1D, 0x21, 0x0C, 0x0B, 0x06, 0x43}), "lcd", "gamma p1");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xF1, {0x56, 0x78, 0x94, 0x2C, 0x2D, 0xAF}), "lcd", "gamma n1");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xF2, {0x1D, 0x21, 0x0C, 0x0B, 0x06, 0x43}), "lcd", "gamma p2");
    ESP_RETURN_ON_ERROR(lcd_cmd_data(0xF3, {0x56, 0x78, 0x94, 0x2C, 0x2D, 0xAF}), "lcd", "gamma n2");
    ESP_RETURN_ON_ERROR(lcd_cmd(0x21), "lcd", "invert on");
    ESP_RETURN_ON_ERROR(lcd_cmd(0x29), "lcd", "display on");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];
    x0 += LCD_X_OFFSET;
    x1 += LCD_X_OFFSET;
    y0 += LCD_Y_OFFSET;
    y1 += LCD_Y_OFFSET;
    ESP_RETURN_ON_ERROR(lcd_cmd(0x2A), "lcd", "caset");
    data[0] = x0 >> 8; data[1] = x0 & 0xFF; data[2] = x1 >> 8; data[3] = x1 & 0xFF;
    ESP_RETURN_ON_ERROR(lcd_data(data, 4), "lcd", "caset data");
    ESP_RETURN_ON_ERROR(lcd_cmd(0x2B), "lcd", "raset");
    data[0] = y0 >> 8; data[1] = y0 & 0xFF; data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    ESP_RETURN_ON_ERROR(lcd_data(data, 4), "lcd", "raset data");
    return lcd_cmd(0x2C);
}

static esp_err_t lcd_fill(uint16_t color)
{
    ESP_RETURN_ON_ERROR(lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1), "lcd", "window");
    constexpr size_t PIXELS = LCD_WIDTH * 20;
    static uint16_t line[PIXELS];
    std::fill(std::begin(line), std::end(line), __builtin_bswap16(color));
    for (int y = 0; y < LCD_HEIGHT; y += 20) {
        ESP_RETURN_ON_ERROR(lcd_data(reinterpret_cast<uint8_t *>(line), sizeof(line)), "lcd", "fill");
    }
    return ESP_OK;
}

static const uint8_t *font5x7(char c)
{
    static const uint8_t sp[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t colon[7] = {0, 0x04, 0x04, 0, 0x04, 0x04, 0};
    static const uint8_t dash[7] = {0, 0, 0, 0x1F, 0, 0, 0};
    static const uint8_t qmark[7] = {0x0E, 0x11, 0x01, 0x06, 0x04, 0, 0x04};
    static const uint8_t n0[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
    static const uint8_t n1[7] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const uint8_t n2[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
    static const uint8_t n3[7] = {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E};
    static const uint8_t n4[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
    static const uint8_t n5[7] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E};
    static const uint8_t n6[7] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E};
    static const uint8_t n7[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    static const uint8_t n8[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
    static const uint8_t n9[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C};
    static const uint8_t A[7] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t B[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
    static const uint8_t C[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    static const uint8_t D[7] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E};
    static const uint8_t E[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    static const uint8_t F[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
    static const uint8_t G[7] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F};
    static const uint8_t H[7] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t I[7] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const uint8_t K[7] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    static const uint8_t L[7] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
    static const uint8_t M[7] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    static const uint8_t N[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    static const uint8_t O[7] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t P[7] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
    static const uint8_t R[7] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    static const uint8_t S[7] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
    static const uint8_t T[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const uint8_t U[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t V[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04};
    static const uint8_t W[7] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A};
    static const uint8_t X[7] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11};
    static const uint8_t Y[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};

    switch (c) {
    case ' ': return sp; case ':': return colon; case '-': return dash;
    case '0': return n0; case '1': return n1; case '2': return n2; case '3': return n3; case '4': return n4;
    case '5': return n5; case '6': return n6; case '7': return n7; case '8': return n8; case '9': return n9;
    case 'A': return A; case 'B': return B; case 'C': return C; case 'D': return D; case 'E': return E;
    case 'F': return F; case 'G': return G; case 'H': return H; case 'I': return I; case 'K': return K;
    case 'L': return L; case 'M': return M; case 'N': return N; case 'O': return O; case 'P': return P;
    case 'R': return R; case 'S': return S; case 'T': return T; case 'U': return U; case 'V': return V;
    case 'W': return W; case 'X': return X; case 'Y': return Y;
    default: return qmark;
    }
}

static esp_err_t lcd_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || w == 0 || h == 0) {
        return ESP_OK;
    }
    w = std::min<uint16_t>(w, LCD_WIDTH - x);
    h = std::min<uint16_t>(h, LCD_HEIGHT - y);
    ESP_RETURN_ON_ERROR(lcd_set_window(x, y, x + w - 1, y + h - 1), "lcd", "rect window");
    std::vector<uint16_t> line(w, __builtin_bswap16(color));
    for (uint16_t row = 0; row < h; ++row) {
        ESP_RETURN_ON_ERROR(lcd_data(reinterpret_cast<uint8_t *>(line.data()), line.size() * sizeof(uint16_t)), "lcd", "rect data");
    }
    return ESP_OK;
}

static esp_err_t lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint8_t scale)
{
    const uint8_t *rows = font5x7(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    for (uint8_t row = 0; row < 7; ++row) {
        for (uint8_t col = 0; col < 5; ++col) {
            if (rows[row] & (1u << (4 - col))) {
                ESP_RETURN_ON_ERROR(lcd_draw_rect(x + col * scale, y + row * scale, scale, scale, fg), "lcd", "char");
            }
        }
    }
    return ESP_OK;
}

static esp_err_t lcd_draw_text(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint8_t scale)
{
    uint16_t cx = x;
    while (*text) {
        ESP_RETURN_ON_ERROR(lcd_draw_char(cx, y, *text++, fg, scale), "lcd", "text");
        cx += 6 * scale;
    }
    return ESP_OK;
}

static esp_err_t lcd_gpio_prepare()
{
    ESP_RETURN_ON_ERROR(s_pca.begin(), "lcd_gpio", "pca begin");

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_LCD_SCLK) | (1ULL << PIN_LCD_MOSI) |
                      (1ULL << PIN_LCD_DC) | (1ULL << PIN_LCD_BL);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&io), "lcd_gpio", "gpio config");

    gpio_set_level(PIN_LCD_SCLK, 0);
    gpio_set_level(PIN_LCD_MOSI, 0);
    gpio_set_level(PIN_LCD_DC, 1);
    gpio_set_level(PIN_LCD_BL, 1);
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_CS, true), "lcd_gpio", "cs high");
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_RST, true), "lcd_gpio", "rst high");
    return ESP_OK;
}

static void lcd_gpio_write_byte(uint8_t value)
{
    for (int bit = 7; bit >= 0; --bit) {
        gpio_set_level(PIN_LCD_SCLK, 0);
        gpio_set_level(PIN_LCD_MOSI, (value >> bit) & 1);
        esp_rom_delay_us(1);
        gpio_set_level(PIN_LCD_SCLK, 1);
        esp_rom_delay_us(1);
    }
    gpio_set_level(PIN_LCD_SCLK, 0);
}

static esp_err_t lcd_gpio_write(bool dc, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    gpio_set_level(PIN_LCD_DC, dc ? 1 : 0);
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_CS, false), "lcd_gpio", "cs low");
    for (size_t i = 0; i < len; ++i) {
        lcd_gpio_write_byte(data[i]);
    }
    return s_pca.writePin(PCA_IO_LCD_CS, true);
}

static esp_err_t lcd_gpio_cmd(uint8_t cmd)
{
    return lcd_gpio_write(false, &cmd, 1);
}

static esp_err_t lcd_gpio_data(const uint8_t *data, size_t len)
{
    return lcd_gpio_write(true, data, len);
}

static esp_err_t lcd_gpio_cmd_data(uint8_t cmd, const std::initializer_list<uint8_t> &data)
{
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(cmd), "lcd_gpio", "cmd");
    if (data.size() > 0) {
        ESP_RETURN_ON_ERROR(lcd_gpio_data(data.begin(), data.size()), "lcd_gpio", "data");
    }
    return ESP_OK;
}

static esp_err_t lcd_gpio_reset()
{
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_CS, true), "lcd_gpio", "cs high");
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_RST, false), "lcd_gpio", "rst low");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_RST, true), "lcd_gpio", "rst high");
    vTaskDelay(pdMS_TO_TICKS(160));
    return ESP_OK;
}

static esp_err_t lcd_gpio_init_gc9307()
{
    ESP_RETURN_ON_ERROR(lcd_gpio_prepare(), "lcd_gpio", "prepare");
    ESP_RETURN_ON_ERROR(lcd_gpio_reset(), "lcd_gpio", "reset");

    // Same GC9307 2.01-inch vendor sequence as the normal SPI path, sent by
    // slow GPIO toggling so a logic-analyzer-free board can still be checked.
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0x11), "lcd_gpio", "sleep out");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0xFE), "lcd_gpio", "unlock");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0xEF), "lcd_gpio", "unlock2");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x36, {0x48}), "lcd_gpio", "madctl");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x3A, {0x05}), "lcd_gpio", "rgb565");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x84, {0x40}), "lcd_gpio", "set 84");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x86, {0x98}), "lcd_gpio", "set 86");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x89, {0x13}), "lcd_gpio", "set 89");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x8B, {0x80}), "lcd_gpio", "set 8b");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x8D, {0x33}), "lcd_gpio", "set 8d");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x8E, {0x0F}), "lcd_gpio", "set 8e");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xB6, {0x00, 0x00, 0x24}), "lcd_gpio", "set b6");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xE8, {0x13, 0x00}), "lcd_gpio", "set e8");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xEC, {0x33, 0x00, 0xF0}), "lcd_gpio", "set ec");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xFF, {0x62}), "lcd_gpio", "set ff");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x99, {0x3E}), "lcd_gpio", "set 99");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x9D, {0x4B}), "lcd_gpio", "set 9d");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x98, {0x3E}), "lcd_gpio", "set 98");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0x9C, {0x4B}), "lcd_gpio", "set 9c");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xC3, {0x2A}), "lcd_gpio", "vreg1a");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xC4, {0x14}), "lcd_gpio", "vreg1b");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xC9, {0x34}), "lcd_gpio", "vreg2a");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xF0, {0x1D, 0x21, 0x0C, 0x0B, 0x06, 0x43}), "lcd_gpio", "gamma p1");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xF1, {0x56, 0x78, 0x94, 0x2C, 0x2D, 0xAF}), "lcd_gpio", "gamma n1");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xF2, {0x1D, 0x21, 0x0C, 0x0B, 0x06, 0x43}), "lcd_gpio", "gamma p2");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd_data(0xF3, {0x56, 0x78, 0x94, 0x2C, 0x2D, 0xAF}), "lcd_gpio", "gamma n2");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0x21), "lcd_gpio", "invert on");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0x11), "lcd_gpio", "sleep out 2");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0x29), "lcd_gpio", "display on");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t lcd_gpio_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];
    x0 += LCD_X_OFFSET;
    x1 += LCD_X_OFFSET;
    y0 += LCD_Y_OFFSET;
    y1 += LCD_Y_OFFSET;
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0x2A), "lcd_gpio", "caset");
    data[0] = x0 >> 8; data[1] = x0 & 0xFF; data[2] = x1 >> 8; data[3] = x1 & 0xFF;
    ESP_RETURN_ON_ERROR(lcd_gpio_data(data, sizeof(data)), "lcd_gpio", "caset data");
    ESP_RETURN_ON_ERROR(lcd_gpio_cmd(0x2B), "lcd_gpio", "raset");
    data[0] = y0 >> 8; data[1] = y0 & 0xFF; data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    ESP_RETURN_ON_ERROR(lcd_gpio_data(data, sizeof(data)), "lcd_gpio", "raset data");
    return lcd_gpio_cmd(0x2C);
}

static esp_err_t lcd_gpio_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || w == 0 || h == 0) {
        return ESP_OK;
    }
    w = std::min<uint16_t>(w, LCD_WIDTH - x);
    h = std::min<uint16_t>(h, LCD_HEIGHT - y);
    ESP_RETURN_ON_ERROR(lcd_gpio_set_window(x, y, x + w - 1, y + h - 1), "lcd_gpio", "window");
    const uint8_t hi = static_cast<uint8_t>(color >> 8);
    const uint8_t lo = static_cast<uint8_t>(color & 0xFF);
    gpio_set_level(PIN_LCD_DC, 1);
    ESP_RETURN_ON_ERROR(s_pca.writePin(PCA_IO_LCD_CS, false), "lcd_gpio", "cs low pixels");
    for (uint32_t i = 0; i < static_cast<uint32_t>(w) * h; ++i) {
        lcd_gpio_write_byte(hi);
        lcd_gpio_write_byte(lo);
        if ((i & 0x3FF) == 0) {
            vTaskDelay(1);
        }
    }
    return s_pca.writePin(PCA_IO_LCD_CS, true);
}

static bool cmd_test_lcd_gpio()
{
    esp_err_t err = lcd_gpio_init_gc9307();
    if (err != ESP_OK) {
        fail("test_lcd_gpio", "GPIO bitbang init failed: " + esp_err_str(err));
        return false;
    }
    gpio_set_level(PIN_LCD_BL, 1);
    info("test_lcd_gpio", "BL=1, sending four vertical color bars by GPIO bitbang");
    const uint16_t w = LCD_WIDTH / 4;
    err = lcd_gpio_draw_rect(0, 0, w, LCD_HEIGHT, 0xF800);
    if (err == ESP_OK) err = lcd_gpio_draw_rect(w, 0, w, LCD_HEIGHT, 0x07E0);
    if (err == ESP_OK) err = lcd_gpio_draw_rect(w * 2, 0, w, LCD_HEIGHT, 0x001F);
    if (err == ESP_OK) err = lcd_gpio_draw_rect(w * 3, 0, LCD_WIDTH - w * 3, LCD_HEIGHT, 0xFFFF);
    if (err != ESP_OK) {
        fail("test_lcd_gpio", "GPIO bitbang draw failed: " + esp_err_str(err));
        return false;
    }
    pass("test_lcd_gpio", "GC9307 init and color bars sent by GPIO bitbang");
    return true;
}

static bool cmd_test_lcd_pins()
{
    esp_err_t err = lcd_gpio_prepare();
    if (err != ESP_OK) {
        fail("test_lcd_pins", "pin prepare failed: " + esp_err_str(err));
        return false;
    }

    struct GpioHold {
        const char *name;
        gpio_num_t pin;
    };
    const GpioHold gpios[] = {
        {"BL", PIN_LCD_BL},
        {"DC", PIN_LCD_DC},
        {"SCLK", PIN_LCD_SCLK},
        {"MOSI", PIN_LCD_MOSI},
    };
    for (const auto &p : gpios) {
        gpio_set_level(p.pin, 0);
        info("test_lcd_pins", std::string(p.name) + "=0 for 3s");
        vTaskDelay(pdMS_TO_TICKS(3000));
        gpio_set_level(p.pin, 1);
        info("test_lcd_pins", std::string(p.name) + "=1 for 3s");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    s_pca.writePin(PCA_IO_LCD_CS, false);
    info("test_lcd_pins", "CS=PCA3 low for 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    s_pca.writePin(PCA_IO_LCD_CS, true);
    info("test_lcd_pins", "CS=PCA3 high for 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    s_pca.writePin(PCA_IO_LCD_RST, false);
    info("test_lcd_pins", "RST=PCA4 low for 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    s_pca.writePin(PCA_IO_LCD_RST, true);
    info("test_lcd_pins", "RST=PCA4 high for 3s");
    vTaskDelay(pdMS_TO_TICKS(3000));

    gpio_set_level(PIN_LCD_BL, 1);
    gpio_set_level(PIN_LCD_DC, 1);
    gpio_set_level(PIN_LCD_SCLK, 0);
    gpio_set_level(PIN_LCD_MOSI, 0);
    s_pca.writePin(PCA_IO_LCD_CS, true);
    s_pca.writePin(PCA_IO_LCD_RST, true);
    pass("test_lcd_pins", "manual LCD pin hold sequence completed");
    return true;
}
#endif

static bool cmd_test_lcd()
{
    esp_err_t err = nv3030b_lcd_init();
    if (err != ESP_OK) {
        fail("test_lcd", "NV3030B SPI init failed: " + esp_err_str(err));
        return false;
    }
    err = nv3030b_lcd_show_test_pattern();
    if (err != ESP_OK) {
        fail("test_lcd", "NV3030B test pattern failed: " + esp_err_str(err));
        return false;
    }
    pass("test_lcd", "NV3030B 240x284 SPI color bars held");
    return true;
}

static bool cmd_test_touch()
{
    if (!i2c_probe(I2C_ADDR_TOUCH)) {
        info("test_touch", "touch controller idle/NACK; continuing because CST816D may answer only after a touch");
    }
    printf("Touch the panel during the next 12 seconds.\n");
    bool got = false;
    int64_t end = esp_timer_get_time() + 12LL * 1000 * 1000;
    while (esp_timer_get_time() < end) {
        uint8_t data[7] = {};
        esp_err_t err = i2c_read_regs(I2C_ADDR_TOUCH, 0x00, data, sizeof(data));
        const uint8_t fingers = data[2] & 0x0F;
        if (err == ESP_OK && fingers) {
            uint16_t x = ((data[3] & 0x0F) << 8) | data[4];
            uint16_t y = ((data[5] & 0x0F) << 8) | data[6];
#if TOUCH_SWAP_XY
            std::swap(x, y);
#endif
#if TOUCH_INVERT_X
            x = LCD_WIDTH - 1 - std::min<uint16_t>(x, LCD_WIDTH - 1);
#endif
#if TOUCH_INVERT_Y
            y = LCD_HEIGHT - 1 - std::min<uint16_t>(y, LCD_HEIGHT - 1);
#endif
            got = true;
            char b[96];
            snprintf(b, sizeof(b), "x=%u y=%u gesture=%u fingers=%u", x, y, data[1], fingers);
            info("test_touch", b);
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    got ? pass("test_touch", "CST816D coordinate data received from 0x15")
        : fail("test_touch", "no coordinate data; check SDA/SCL and touch power");
    return got;
}

static bool cmd_test_sd()
{
    if (!s_sd_mounted) {
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
        mount_config.format_if_mount_failed = false;
        // Runtime keeps sensor, measurement and room-survey logs open, while
        // photos/DXF exports need temporary handles. Eight slots caused the
        // two ROOM files to contain only headers and reject every point write.
        mount_config.max_files = 16;
        mount_config.allocation_unit_size = 16 * 1024;
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = SDMMC_FREQ_DEFAULT;
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        slot_config.width = 1;
        slot_config.clk = PIN_SD_CLK;
        slot_config.cmd = PIN_SD_CMD;
        slot_config.d0 = PIN_SD_D0;
        slot_config.d1 = GPIO_NUM_NC;
        slot_config.d2 = GPIO_NUM_NC;
        slot_config.d3 = GPIO_NUM_NC;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
        esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_sd_card);
        if (err != ESP_OK) {
            fail("test_sd", "mount failed: " + esp_err_str(err) + "; check card, CMD/CLK/D0, pull-ups, filesystem");
            return false;
        }
        s_sd_mounted = true;
    }
    uint64_t cap_mb = (static_cast<uint64_t>(s_sd_card->csd.capacity) * s_sd_card->csd.sector_size) / (1024 * 1024);
    std::string path = std::string(SD_MOUNT_POINT) + "/selftest.txt";
    std::string line = "board_self_test uptime_ms=" + std::to_string(esp_timer_get_time() / 1000) + "\n";
    FILE *f = fopen(path.c_str(), "w");
    if (!f) {
        fail("test_sd", std::string("open for write failed errno=") + std::to_string(errno));
        return false;
    }
    fwrite(line.data(), 1, line.size(), f);
    fclose(f);
    char readback[128] = {};
    f = fopen(path.c_str(), "r");
    if (!f) {
        fail("test_sd", std::string("open for read failed errno=") + std::to_string(errno));
        return false;
    }
    size_t n = fread(readback, 1, sizeof(readback) - 1, f);
    fclose(f);
    bool ok = line == std::string(readback, n);
    ok ? pass("test_sd", "mounted capacity=" + std::to_string(cap_mb) + "MB write/read OK path=/selftest.txt")
       : fail("test_sd", "readback mismatch");
    return ok;
}

static bool cmd_test_sd_speed()
{
    if (!s_sd_mounted && !cmd_test_sd()) {
        fail("test_sd_speed", "SD mount/read-write baseline failed");
        return false;
    }

    static constexpr size_t block_size = 16 * 1024;
    static constexpr size_t total_size = 4 * 1024 * 1024;
    std::string path = std::string(SD_MOUNT_POINT) + "/sdspd.bin";
    uint8_t *buf = static_cast<uint8_t *>(heap_caps_malloc(block_size, MALLOC_CAP_8BIT));
    if (!buf) {
        fail("test_sd_speed", "buffer alloc failed");
        return false;
    }
    for (size_t i = 0; i < block_size; ++i) {
        buf[i] = static_cast<uint8_t>((i * 31u + 7u) & 0xFFu);
    }

    FILE *f = fopen(path.c_str(), "wb");
    if (!f) {
        heap_caps_free(buf);
        fail("test_sd_speed", std::string("open write failed errno=") + std::to_string(errno));
        return false;
    }
    int64_t t0 = esp_timer_get_time();
    size_t written = 0;
    while (written < total_size) {
        size_t n = fwrite(buf, 1, block_size, f);
        if (n != block_size) {
            fclose(f);
            heap_caps_free(buf);
            fail("test_sd_speed", std::string("write failed at=") + std::to_string(written) + " errno=" + std::to_string(errno));
            return false;
        }
        written += n;
    }
    fflush(f);
    fclose(f);
    int64_t write_us = esp_timer_get_time() - t0;

    f = fopen(path.c_str(), "rb");
    if (!f) {
        heap_caps_free(buf);
        fail("test_sd_speed", std::string("open read failed errno=") + std::to_string(errno));
        return false;
    }
    t0 = esp_timer_get_time();
    size_t read = 0;
    uint32_t checksum = 0;
    while (read < total_size) {
        size_t n = fread(buf, 1, block_size, f);
        if (n == 0) {
            break;
        }
        for (size_t i = 0; i < n; i += 257) {
            checksum += buf[i];
        }
        read += n;
    }
    fclose(f);
    int64_t read_us = esp_timer_get_time() - t0;
    remove(path.c_str());
    heap_caps_free(buf);

    if (read != total_size) {
        fail("test_sd_speed", "read size mismatch read=" + std::to_string(read) + " expected=" + std::to_string(total_size));
        return false;
    }

    double write_mbps = (static_cast<double>(written) * 1000000.0) / (1024.0 * 1024.0 * static_cast<double>(write_us));
    double read_mbps = (static_cast<double>(read) * 1000000.0) / (1024.0 * 1024.0 * static_cast<double>(read_us));
    char detail[160];
    snprintf(detail, sizeof(detail), "size=%uKB block=%uKB write=%.2fMB/s read=%.2fMB/s checksum=%lu",
             static_cast<unsigned>(total_size / 1024),
             static_cast<unsigned>(block_size / 1024),
             write_mbps, read_mbps, static_cast<unsigned long>(checksum));
    pass("test_sd_speed", detail);
    return true;
}

static bool cmd_test_camera()
{
    if (s_camera_http_ready) {
        fail("test_camera", "camera HTTP preview is running; use stop_camera_ap first");
        return false;
    }
    camera_config_t config = {};
    config.pin_pwdn = -1;
    config.pin_reset = PIN_CAM_RST;
    config.pin_xclk = PIN_CAM_XCLK;
    config.pin_d7 = PIN_CAM_D7;
    config.pin_d6 = PIN_CAM_D6;
    config.pin_d5 = PIN_CAM_D5;
    config.pin_d4 = PIN_CAM_D4;
    config.pin_d3 = PIN_CAM_D3;
    config.pin_d2 = PIN_CAM_D2;
    config.pin_d1 = PIN_CAM_D1;
    config.pin_d0 = PIN_CAM_D0;
    config.pin_vsync = PIN_CAM_VSYNC;
    config.pin_href = PIN_CAM_HREF;
    config.pin_pclk = PIN_CAM_PCLK;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_1;
    config.ledc_channel = LEDC_CHANNEL_1;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = esp_psram_is_initialized() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = camera_use_shared_i2c(config);
    if (err != ESP_OK) {
        fail("test_camera", "camera SCCB init failed: " + esp_err_str(err));
        return false;
    }

    esp_camera_deinit();
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
        fail("test_camera", "esp_camera_init failed: " + esp_err_str(err) + "; check SCCB 2.8V/PCA9306 and DVP pins");
        esp_camera_deinit();
        return false;
    }
    sensor_t *sensor = esp_camera_sensor_get();
    std::string sid = sensor ? ("pid=0x" + hex_addr(sensor->id.PID).substr(2) + " ver=0x" + hex_addr(sensor->id.VER).substr(2)) : "sensor id unavailable";
    bool ok = true;
    size_t last_len = 0;
    int64_t total_ms = 0;
    bool saved_jpg = false;
    for (int i = 0; i < 10; ++i) {
        int64_t t0 = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        int64_t dt = (esp_timer_get_time() - t0) / 1000;
        if (!fb || fb->len == 0) {
            ok = false;
            info("test_camera", "frame " + std::to_string(i) + " empty dt_ms=" + std::to_string(dt));
            if (fb) esp_camera_fb_return(fb);
            continue;
        }
        last_len = fb->len;
        total_ms += dt;
        info("test_camera", "frame " + std::to_string(i) + " len=" + std::to_string(fb->len) + " dt_ms=" + std::to_string(dt));
        if (!saved_jpg && s_sd_mounted) {
            std::string cam_path = std::string(SD_MOUNT_POINT) + "/cam_test.jpg";
            FILE *f = fopen(cam_path.c_str(), "wb");
            if (f) {
                fwrite(fb->buf, 1, fb->len, f);
                fclose(f);
                info("test_camera", "saved /cam_test.jpg");
                saved_jpg = true;
            } else {
                info("test_camera", "SD mounted but /cam_test.jpg write failed");
            }
        }
        esp_camera_fb_return(fb);
    }
    ok ? pass("test_camera", sid + " frames=10 last_len=" + std::to_string(last_len) + " avg_ms=" + std::to_string(total_ms / 10))
       : fail("test_camera", sid + " frame capture failed; check PCLK/HREF/VSYNC/D0-D7 and XCLK");
    esp_camera_deinit();
    return ok;
}

static bool cmd_stream_camera_uart(const std::string &cmd)
{
    if (s_camera_http_ready) {
        fail("stream_camera_uart", "camera HTTP preview is running; use stop_camera_ap first");
        return false;
    }
    int frames = 120;
    size_t pos = cmd.find(' ');
    if (pos != std::string::npos) {
        int parsed = std::strtol(cmd.c_str() + pos + 1, nullptr, 10);
        if (parsed > 0 && parsed <= 2000) {
            frames = parsed;
        }
    }

    camera_config_t config = {};
    config.pin_pwdn = -1;
    config.pin_reset = PIN_CAM_RST;
    config.pin_xclk = PIN_CAM_XCLK;
    config.pin_d7 = PIN_CAM_D7;
    config.pin_d6 = PIN_CAM_D6;
    config.pin_d5 = PIN_CAM_D5;
    config.pin_d4 = PIN_CAM_D4;
    config.pin_d3 = PIN_CAM_D3;
    config.pin_d2 = PIN_CAM_D2;
    config.pin_d1 = PIN_CAM_D1;
    config.pin_d0 = PIN_CAM_D0;
    config.pin_vsync = PIN_CAM_VSYNC;
    config.pin_href = PIN_CAM_HREF;
    config.pin_pclk = PIN_CAM_PCLK;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_1;
    config.ledc_channel = LEDC_CHANNEL_1;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = esp_psram_is_initialized() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = camera_use_shared_i2c(config);
    if (err != ESP_OK) {
        fail("stream_camera_uart", "camera SCCB init failed: " + esp_err_str(err));
        return false;
    }

    esp_log_level_set("*", ESP_LOG_NONE);
    esp_camera_deinit();
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
        esp_log_level_set("*", ESP_LOG_INFO);
        fail("stream_camera_uart", "esp_camera_init failed: " + esp_err_str(err));
        esp_camera_deinit();
        return false;
    }

    printf("#CAMUART BEGIN frames=%d format=rgb565_to_jpeg size=qqvga baud=115200\n", frames);
    fflush(stdout);
    int sent = 0;
    for (int i = 0; i < frames; ++i) {
        int64_t t0 = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        int64_t dt = (esp_timer_get_time() - t0) / 1000;
        if (!fb || fb->len == 0) {
            printf("#ERR frame=%d dt_ms=%lld\n", i, static_cast<long long>(dt));
            if (fb) {
                esp_camera_fb_return(fb);
            }
            continue;
        }

        uint8_t *jpg = nullptr;
        size_t jpg_len = 0;
        bool converted = frame2jpg(fb, 75, &jpg, &jpg_len);
        esp_camera_fb_return(fb);
        if (!converted || !jpg || jpg_len == 0) {
            printf("#ERR frame=%d convert_failed dt_ms=%lld\n", i, static_cast<long long>(dt));
            if (jpg) {
                free(jpg);
            }
            continue;
        }

        printf("#JPG seq=%d len=%u dt_ms=%lld\n", i, static_cast<unsigned>(jpg_len), static_cast<long long>(dt));
        fflush(stdout);
        uart_write_bytes(UART_NUM_0, reinterpret_cast<const char *>(jpg), jpg_len);
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(5000));
        printf("\n#END seq=%d\n", i);
        fflush(stdout);
        free(jpg);
        ++sent;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    printf("#CAMUART DONE sent=%d\n", sent);
    esp_camera_deinit();
    esp_log_level_set("*", ESP_LOG_INFO);
    pass("stream_camera_uart", "sent JPEG frames=" + std::to_string(sent));
    return sent > 0;
}

static constexpr uint8_t BNO_CH_COMMAND = 0;
static constexpr uint8_t BNO_CH_EXECUTABLE = 1;
static constexpr uint8_t BNO_CH_CONTROL = 2;
static constexpr uint8_t BNO_CH_INPUT = 3;
static constexpr uint8_t BNO_CH_WAKE_INPUT = 4;
static constexpr uint8_t BNO_CH_GYRO_RV = 5;

static constexpr uint8_t BNO_RPT_ACCEL = 0x01;
static constexpr uint8_t BNO_RPT_GYRO = 0x02;
static constexpr uint8_t BNO_RPT_MAG = 0x03;
static constexpr uint8_t BNO_RPT_LINEAR_ACCEL = 0x04;
static constexpr uint8_t BNO_RPT_ROTATION_VECTOR = 0x05;
static constexpr uint8_t BNO_RPT_GAME_RV = 0x08;
static constexpr uint8_t BNO_RPT_TIMEBASE = 0xFB;
static constexpr uint8_t BNO_RPT_GET_FEATURE_RESP = 0xFC;
static constexpr uint8_t BNO_RPT_SET_FEATURE = 0xFD;
static constexpr uint8_t BNO_RPT_PRODUCT_ID_RESP = 0xF8;
static constexpr uint8_t BNO_RPT_PRODUCT_ID_REQ = 0xF9;
// This board's BNO086 reliably supplies a complete SHTP packet when clocked
// in one transaction.  Short header-only transactions make this particular
// module hold the bus until the controller times out, so retain a full-size
// read and control bus load through the enabled report rates instead.
static constexpr size_t BNO_I2C_READ_SIZE = 256;

struct BnoPacket {
    uint8_t channel = 0;
    uint8_t sequence = 0;
    bool continuation = false;
    std::vector<uint8_t> payload;
};

struct BnoTestState {
    bool got_packet = false;
    bool got_product = false;
    bool got_feature_response = false;
    bool got_accel = false;
    bool got_gyro = false;
    bool got_mag = false;
    bool got_rotation_vector = false;
    bool got_game_rv = false;
    uint32_t packets = 0;
    uint32_t timebase_reports = 0;
    uint32_t motion_reports = 0;
    uint32_t unknown_reports = 0;
    uint32_t empty_packets = 0;
    uint32_t read_errors = 0;
    uint32_t int_low_count = 0;
    int motion_logs = 0;
    std::string product_detail;
};

static std::array<uint8_t, 6> s_bno_tx_seq = {};

static uint16_t bno_u16(const std::vector<uint8_t> &p, size_t off)
{
    return static_cast<uint16_t>(p[off]) | (static_cast<uint16_t>(p[off + 1]) << 8);
}

static int16_t bno_s16(const std::vector<uint8_t> &p, size_t off)
{
    return static_cast<int16_t>(bno_u16(p, off));
}

static uint32_t bno_u32(const std::vector<uint8_t> &p, size_t off)
{
    return static_cast<uint32_t>(p[off]) |
           (static_cast<uint32_t>(p[off + 1]) << 8) |
           (static_cast<uint32_t>(p[off + 2]) << 16) |
           (static_cast<uint32_t>(p[off + 3]) << 24);
}

static float q_to_float(int16_t q, uint8_t point)
{
    return static_cast<float>(q) / static_cast<float>(1UL << point);
}

static const char *bno_report_name(uint8_t report_id)
{
    switch (report_id) {
    case BNO_RPT_ACCEL: return "accelerometer";
    case BNO_RPT_GYRO: return "gyroscope";
    case BNO_RPT_MAG: return "magnetic_field";
    case BNO_RPT_ROTATION_VECTOR: return "rotation_vector";
    case BNO_RPT_GAME_RV: return "game_rotation_vector";
    case BNO_RPT_TIMEBASE: return "timebase";
    case BNO_RPT_GET_FEATURE_RESP: return "get_feature_response";
    case BNO_RPT_PRODUCT_ID_RESP: return "product_id_response";
    default: return "unknown";
    }
}

static bool bno_wait_for_int(int timeout_ms)
{
    int64_t end = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
    do {
        if (gpio_get_level(PIN_BNO086_INT) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (esp_timer_get_time() < end);
    return false;
}

static esp_err_t bno_write_packet(uint8_t channel, const uint8_t *payload, size_t len)
{
    if (channel >= s_bno_tx_seq.size() || len > 32762) {
        return ESP_ERR_INVALID_ARG;
    }
    std::vector<uint8_t> pkt(4 + len);
    uint16_t total = static_cast<uint16_t>(4 + len);
    pkt[0] = total & 0xFF;
    pkt[1] = total >> 8;
    pkt[2] = channel;
    pkt[3] = s_bno_tx_seq[channel]++;
    memcpy(pkt.data() + 4, payload, len);
    return i2c_write_raw(s_bno_addr, pkt.data(), pkt.size(), 120);
}

static esp_err_t bno_read_packet(BnoPacket &pkt)
{
    // A single transaction is intentional here.  On the fitted BNO086,
    // ending a transaction after only the four-byte header causes the next
    // request to clock-stretch until the ESP32 I2C hardware times out.
    if (!i2c_take_bus(400)) {
        return ESP_ERR_TIMEOUT;
    }
    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t err = i2c_add_temp_device(s_bno_addr, &dev);
    std::array<uint8_t, BNO_I2C_READ_SIZE> buf = {};
    if (err == ESP_OK) {
        err = i2c_master_receive(dev, buf.data(), buf.size(), 260);
    }
    if (dev) i2c_master_bus_rm_device(dev);
    i2c_give_bus();
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t len = static_cast<uint16_t>(buf[0]) |
                         (static_cast<uint16_t>(buf[1] & 0x7F) << 8);
    if (len == 0 || (len == 4 && buf[2] == 0)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (len < 4 || len == 0x7FFF || len > buf.size()) {
        return ESP_ERR_INVALID_SIZE;
    }

    pkt.channel = buf[2];
    pkt.sequence = buf[3];
    pkt.continuation = (buf[1] & 0x80) != 0;
    pkt.payload.assign(buf.begin() + 4, buf.begin() + len);
    return ESP_OK;
}

static esp_err_t bno_read_packet_wait(BnoPacket &pkt, int timeout_ms)
{
    if (!bno_wait_for_int(timeout_ms)) {
        return ESP_ERR_TIMEOUT;
    }
    return bno_read_packet(pkt);
}

static esp_err_t bno_soft_reset()
{
    std::fill(s_bno_tx_seq.begin(), s_bno_tx_seq.end(), 0);
    uint8_t reset = 0x01;
    esp_err_t err = bno_write_packet(BNO_CH_EXECUTABLE, &reset, 1);
    vTaskDelay(pdMS_TO_TICKS(650));
    std::fill(s_bno_tx_seq.begin(), s_bno_tx_seq.end(), 0);
    return err;
}

static esp_err_t bno_enable_feature(uint8_t report_id, uint32_t interval_us)
{
    uint8_t p[17] = {};
    p[0] = BNO_RPT_SET_FEATURE;
    p[1] = report_id;
    p[5] = interval_us & 0xFF;
    p[6] = (interval_us >> 8) & 0xFF;
    p[7] = (interval_us >> 16) & 0xFF;
    p[8] = (interval_us >> 24) & 0xFF;
    return bno_write_packet(BNO_CH_CONTROL, p, sizeof(p));
}

static void bno_log_motion(BnoTestState &st, const char *text)
{
    if (st.motion_logs < 16) {
        info("test_bno086", text);
        ++st.motion_logs;
    }
}

static size_t bno_parse_motion_report(const std::vector<uint8_t> &p, size_t off, BnoTestState &st)
{
    uint8_t rid = p[off];
    size_t remain = p.size() - off;
    if ((rid == BNO_RPT_ACCEL || rid == BNO_RPT_GYRO || rid == BNO_RPT_MAG) && remain >= 10) {
        uint8_t status = p[off + 2] & 0x03;
        int16_t x_raw = bno_s16(p, off + 4);
        int16_t y_raw = bno_s16(p, off + 6);
        int16_t z_raw = bno_s16(p, off + 8);
        char b[220];
        if (rid == BNO_RPT_ACCEL) {
            float ax = q_to_float(x_raw, 8);
            float ay = q_to_float(y_raw, 8);
            float az = q_to_float(z_raw, 8);
            float norm = std::sqrt(ax * ax + ay * ay + az * az);
            snprintf(b, sizeof(b), "accel ax=%.3f ay=%.3f az=%.3f norm=%.3f m/s2 status=%u seq=%u",
                     ax, ay, az, norm, status, p[off + 1]);
            st.got_accel = true;
        } else if (rid == BNO_RPT_GYRO) {
            float gx = q_to_float(x_raw, 9);
            float gy = q_to_float(y_raw, 9);
            float gz = q_to_float(z_raw, 9);
            snprintf(b, sizeof(b), "gyro gx=%.4f gy=%.4f gz=%.4f rad/s status=%u seq=%u",
                     gx, gy, gz, status, p[off + 1]);
            st.got_gyro = true;
        } else {
            float mx = q_to_float(x_raw, 4);
            float my = q_to_float(y_raw, 4);
            float mz = q_to_float(z_raw, 4);
            snprintf(b, sizeof(b), "mag mx=%.3f my=%.3f mz=%.3f uT status=%u seq=%u",
                     mx, my, mz, status, p[off + 1]);
            st.got_mag = true;
        }
        ++st.motion_reports;
        bno_log_motion(st, b);
        return 10;
    }

    if ((rid == BNO_RPT_GAME_RV || rid == BNO_RPT_ROTATION_VECTOR) && remain >= 12) {
        uint8_t status = p[off + 2] & 0x03;
        int16_t qi = bno_s16(p, off + 4);
        int16_t qj = bno_s16(p, off + 6);
        int16_t qk = bno_s16(p, off + 8);
        int16_t qr = bno_s16(p, off + 10);
        float acc_rad = NAN;
        size_t used = 12;
        if (remain >= 14) {
            acc_rad = q_to_float(bno_s16(p, off + 12), 12);
            used = 14;
        }
        char b[240];
        snprintf(b, sizeof(b), "%s qi=%.4f qj=%.4f qk=%.4f qr=%.4f acc_rad=%.4f status=%u seq=%u",
                 rid == BNO_RPT_GAME_RV ? "game_rv" : "rotation_vector",
                 q_to_float(qi, 14), q_to_float(qj, 14), q_to_float(qk, 14), q_to_float(qr, 14),
                 acc_rad, status, p[off + 1]);
        if (rid == BNO_RPT_GAME_RV) {
            st.got_game_rv = true;
        } else {
            st.got_rotation_vector = true;
        }
        ++st.motion_reports;
        bno_log_motion(st, b);
        return used;
    }

    return 0;
}

static void bno_process_packet(const BnoPacket &pkt, BnoTestState &st, bool verbose_unknown)
{
    st.got_packet = true;
    ++st.packets;
    if (pkt.payload.empty()) {
        return;
    }

    if (pkt.channel == BNO_CH_CONTROL) {
        uint8_t rid = pkt.payload[0];
        if (rid == BNO_RPT_PRODUCT_ID_RESP && pkt.payload.size() >= 16) {
            uint32_t part = bno_u32(pkt.payload, 4);
            uint32_t build = bno_u32(pkt.payload, 8);
            uint16_t patch = bno_u16(pkt.payload, 12);
            char b[220];
            snprintf(b, sizeof(b), "product reset_cause=%u sw=%u.%u.%u part=0x%08lX build=%lu",
                     pkt.payload[1], pkt.payload[2], pkt.payload[3], patch,
                     static_cast<unsigned long>(part), static_cast<unsigned long>(build));
            st.product_detail = b;
            st.got_product = true;
            info("test_bno086", b);
        } else if (rid == BNO_RPT_GET_FEATURE_RESP && pkt.payload.size() >= 17) {
            uint8_t feature = pkt.payload[1];
            uint32_t interval = bno_u32(pkt.payload, 5);
            char b[180];
            snprintf(b, sizeof(b), "feature_response feature=0x%02X (%s) interval_us=%lu flags=0x%02X",
                     feature, bno_report_name(feature), static_cast<unsigned long>(interval), pkt.payload[2]);
            st.got_feature_response = true;
            info("test_bno086", b);
        } else if (verbose_unknown) {
            info("test_bno086", "control channel report=0x" + hex_addr(rid).substr(2) + " len=" + std::to_string(pkt.payload.size()));
        }
        return;
    }

    if (pkt.channel != BNO_CH_INPUT && pkt.channel != BNO_CH_WAKE_INPUT && pkt.channel != BNO_CH_GYRO_RV) {
        if (verbose_unknown) {
            info("test_bno086", "channel=" + std::to_string(pkt.channel) + " seq=" + std::to_string(pkt.sequence) +
                 " len=" + std::to_string(pkt.payload.size()));
        }
        return;
    }

    size_t off = 0;
    while (off < pkt.payload.size()) {
        uint8_t rid = pkt.payload[off];
        if (rid == BNO_RPT_TIMEBASE) {
            if (off + 5 > pkt.payload.size()) {
                break;
            }
            ++st.timebase_reports;
            off += 5;
            continue;
        }
        size_t used = bno_parse_motion_report(pkt.payload, off, st);
        if (used == 0) {
            ++st.unknown_reports;
            if (verbose_unknown) {
                info("test_bno086", "input report=0x" + hex_addr(rid).substr(2) + " (" + bno_report_name(rid) +
                     ") len_remaining=" + std::to_string(pkt.payload.size() - off));
            }
            break;
        }
        off += used;
    }
}

static void bno_drain_packets(BnoTestState &st, int total_ms, bool verbose_unknown)
{
    int64_t end = esp_timer_get_time() + static_cast<int64_t>(total_ms) * 1000;
    while (esp_timer_get_time() < end) {
        BnoPacket pkt;
        esp_err_t err = bno_read_packet_wait(pkt, 30);
        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND) {
            if (err == ESP_ERR_NOT_FOUND) {
                ++st.empty_packets;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (err != ESP_OK) {
            ++st.read_errors;
            continue;
        }
        ++st.int_low_count;
        bno_process_packet(pkt, st, verbose_unknown);
    }
}

static void bno_release_camera_if_needed()
{
    if (!s_camera_httpd && !s_camera_http_ready) {
        return;
    }

    info("test_bno086", "stopping camera preview before BNO086 I2C/SHTP test");
    if (s_camera_httpd) {
        httpd_stop(s_camera_httpd);
        s_camera_httpd = nullptr;
    }
    if (s_stream_httpd) {
        s_stream_stop_requested = true;
        httpd_stop(s_stream_httpd);
        s_stream_httpd = nullptr;
    }
    if (s_camera_http_ready) {
        bool locked = false;
        if (s_camera_mutex) {
            locked = xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(3000)) == pdTRUE;
        }
        esp_camera_deinit();
        s_camera_http_ready = false;
        if (locked) {
            xSemaphoreGive(s_camera_mutex);
        }
    }
    i2c_release_bus();
    vTaskDelay(pdMS_TO_TICKS(30));
}

static bool cmd_test_bno086()
{
    bno_release_camera_if_needed();

    esp_err_t err = i2c_init_once();
    if (err != ESP_OK) {
        fail("test_bno086", "I2C init failed: " + esp_err_str(err));
        return false;
    }

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_BNO086_INT);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&cfg);

    if (!bno_select_i2c_addr()) {
        fail("test_bno086", "0x4A/0x4B not found on I2C; check VDDIO, SDA/SCL, SA0, PS0/PS1, reset, and soldering");
        return false;
    }
    info("test_bno086", "using address " + hex_addr(s_bno_addr));

    BnoTestState st;
    err = bno_soft_reset();
    if (err != ESP_OK) {
        fail("test_bno086", "soft reset write failed: " + esp_err_str(err));
        return false;
    }

    bno_drain_packets(st, 900, false);

    uint8_t product_req[2] = {BNO_RPT_PRODUCT_ID_REQ, 0x00};
    err = bno_write_packet(BNO_CH_CONTROL, product_req, sizeof(product_req));
    if (err != ESP_OK) {
        fail("test_bno086", "product id request failed: " + esp_err_str(err));
        return false;
    }
    bno_drain_packets(st, 500, true);

    struct FeatureReq {
        uint8_t report_id;
        const char *name;
    };
    const FeatureReq features[] = {
        {BNO_RPT_ACCEL, "accelerometer"},
        {BNO_RPT_GYRO, "gyroscope"},
        {BNO_RPT_MAG, "magnetometer"},
        {BNO_RPT_ROTATION_VECTOR, "magnetic rotation vector"},
    };
    for (const auto &f : features) {
        err = bno_enable_feature(f.report_id, 20000);
        if (err != ESP_OK) {
            fail("test_bno086", std::string("set feature failed for ") + f.name + ": " + esp_err_str(err));
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(40));
        bno_drain_packets(st, 180, false);
    }

    int64_t end = esp_timer_get_time() + 7LL * 1000 * 1000;
    while (esp_timer_get_time() < end &&
           !(st.got_accel && st.got_gyro && st.got_mag && st.got_rotation_vector)) {
        BnoPacket pkt;
        err = bno_read_packet_wait(pkt, 250);
        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND) {
            if (err == ESP_ERR_NOT_FOUND) {
                ++st.empty_packets;
            }
            continue;
        }
        if (err != ESP_OK) {
            ++st.read_errors;
            continue;
        }
        ++st.int_low_count;
        bno_process_packet(pkt, st, true);
    }

    char b[260];
    snprintf(b, sizeof(b), "packets=%lu empty=%lu timebase=%lu motion=%lu feature_resp=%s product=%s read_errors=%lu missing:%s%s%s%s",
             static_cast<unsigned long>(st.packets),
             static_cast<unsigned long>(st.empty_packets),
             static_cast<unsigned long>(st.timebase_reports),
             static_cast<unsigned long>(st.motion_reports),
             st.got_feature_response ? "yes" : "no",
             st.got_product ? "yes" : "no",
             static_cast<unsigned long>(st.read_errors),
             st.got_accel ? "" : " accel",
             st.got_gyro ? "" : " gyro",
             st.got_mag ? "" : " mag",
             st.got_rotation_vector ? "" : " rotation_vector");

    if (st.got_accel && st.got_gyro && st.got_mag && st.got_rotation_vector) {
        pass("test_bno086", "addr=" + hex_addr(s_bno_addr) + " " + std::string(b) + "; " + (st.product_detail.empty() ? "product unavailable" : st.product_detail));
        return true;
    }
    fail("test_bno086", st.got_packet ? b : "no SHTP packet after reset; BNO086 does not support I2C polling, check HOST_INTN GPIO and power/reset");
    return false;
}

static esp_err_t uart_init_once(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud, bool *flag)
{
    if (*flag) {
        return ESP_OK;
    }
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_RETURN_ON_ERROR(uart_driver_install(port, 2048, 0, 0, nullptr, 0), "uart", "driver");
    ESP_RETURN_ON_ERROR(uart_param_config(port, &cfg), "uart", "param");
    ESP_RETURN_ON_ERROR(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "uart", "pins");
    // A powered LVTTL transmitter idles high.  The weak pull-up makes a
    // disconnected module deterministic and lets diagnostics distinguish a
    // floating harness from a line actively held low without loading the L1.
    gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);
    *flag = true;
    return ESP_OK;
}

static esp_err_t uart_reconfigure(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud, bool *flag)
{
    if (*flag) {
        uart_driver_delete(port);
        *flag = false;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return uart_init_once(port, tx, rx, baud, flag);
}

static esp_err_t service_nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), "service", "erase NVS");
        err = nvs_flash_init();
    }
    return err;
}

static bool measurement_reserve_session_ids_at_boot()
{
    if (s_measure_session_next != 0) return true;
    if (service_nvs_init() != ESP_OK) return false;
    nvs_handle_t handle = 0;
    if (nvs_open(MEASURE_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
    uint32_t previous = 0;
    esp_err_t err = nvs_get_u32(handle, MEASURE_NVS_COUNTER_KEY, &previous);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    uint32_t first = 1;
    uint32_t limit = MEASURE_SESSION_ID_RESERVATION;
    if (previous > 0 && previous <= UINT32_MAX - MEASURE_SESSION_ID_RESERVATION) {
        first = previous + 1;
        limit = previous + MEASURE_SESSION_ID_RESERVATION;
    }
    // Persist the end of the reservation, not each individual measurement.
    // A sudden power loss can skip unused IDs but can never duplicate them.
    if (err == ESP_OK) err = nvs_set_u32(handle, MEASURE_NVS_COUNTER_KEY, limit);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) return false;
    portENTER_CRITICAL(&s_measure_session_mux);
    s_measure_session_next = first;
    s_measure_session_limit = limit;
    portEXIT_CRITICAL(&s_measure_session_mux);
    ESP_LOGI("measure", "reserved session IDs %lu..%lu before PSRAM tasks start",
             static_cast<unsigned long>(first), static_cast<unsigned long>(limit));
    return true;
}

static uint32_t measurement_allocate_session_id()
{
    uint32_t id = 0;
    portENTER_CRITICAL(&s_measure_session_mux);
    if (s_measure_session_next != 0 && s_measure_session_next <= s_measure_session_limit) {
        id = s_measure_session_next++;
    }
    portEXIT_CRITICAL(&s_measure_session_mux);
    return id;
}

static bool room_reserve_session_ids_at_boot()
{
    if (s_room_session_next != 0) return true;
    if (service_nvs_init() != ESP_OK) return false;
    nvs_handle_t handle = 0;
    if (nvs_open(ROOM_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
    uint32_t previous = 0;
    esp_err_t err = nvs_get_u32(handle, ROOM_NVS_COUNTER_KEY, &previous);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    uint32_t first = 1;
    uint32_t limit = ROOM_SESSION_ID_RESERVATION;
    if (previous > 0 && previous <= UINT32_MAX - ROOM_SESSION_ID_RESERVATION) {
        first = previous + 1;
        limit = previous + ROOM_SESSION_ID_RESERVATION;
    }
    if (err == ESP_OK) err = nvs_set_u32(handle, ROOM_NVS_COUNTER_KEY, limit);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) return false;
    portENTER_CRITICAL(&s_room_session_mux);
    s_room_session_next = first;
    s_room_session_limit = limit;
    portEXIT_CRITICAL(&s_room_session_mux);
    ESP_LOGI("room", "reserved session IDs %lu..%lu before PSRAM tasks start",
             static_cast<unsigned long>(first), static_cast<unsigned long>(limit));
    return true;
}

static uint32_t room_allocate_session_id()
{
    uint32_t id = 0;
    portENTER_CRITICAL(&s_room_session_mux);
    if (s_room_session_next != 0 && s_room_session_next <= s_room_session_limit) {
        id = s_room_session_next++;
    }
    portEXIT_CRITICAL(&s_room_session_mux);
    return id;
}

static std::string printable_raw(const uint8_t *data, int len)
{
    std::string s;
    for (int i = 0; i < len; ++i) {
        uint8_t c = data[i];
        if (c >= 32 && c <= 126) {
            s.push_back(static_cast<char>(c));
        } else if (c == '\r') {
            s += "\\r";
        } else if (c == '\n') {
            s += "\\n";
        } else {
            char b[6];
            snprintf(b, sizeof(b), "\\x%02X", c);
            s += b;
        }
    }
    return s;
}

struct LaserParseResult {
    bool found = false;
    int32_t distance_mm = -1;
    std::string source;
};

static std::string laser_ascii_from_bytes(const std::vector<uint8_t> &bytes)
{
    std::string text;
    text.reserve(bytes.size());
    for (uint8_t b : bytes) {
        if ((b >= 32 && b <= 126) || b == '\r' || b == '\n' || b == '\t') {
            text.push_back(static_cast<char>(b));
        } else {
            text.push_back(' ');
        }
    }
    return text;
}

static std::string upper_copy(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

static bool laser_line_is_command_or_status(const std::string &line)
{
    std::string up = upper_copy(line);
    return up.find("IGET") != std::string::npos ||
           up.find("ISM") != std::string::npos ||
           up.find("IACM") != std::string::npos ||
           up.find("IFACM") != std::string::npos ||
           up.find("IHALT") != std::string::npos ||
           up.find("ILD") != std::string::npos ||
           up.find("BUSY") != std::string::npos ||
           up.find("ERR") != std::string::npos ||
           up.find("ERROR") != std::string::npos;
}

static bool laser_accept_mm(double mm)
{
    return std::isfinite(mm) && mm >= 20.0 && mm <= 200000.0;
}

static bool laser_parse_ascii_line(const std::string &line, LaserParseResult &result)
{
    if (laser_line_is_command_or_status(line)) {
        return false;
    }

    bool found = false;
    int32_t last_mm = -1;
    int best_score = -1;
    for (size_t i = 0; i < line.size(); ++i) {
        unsigned char c = static_cast<unsigned char>(line[i]);
        bool start = std::isdigit(c) || line[i] == '-' || line[i] == '+' ||
                     (line[i] == '.' && i + 1 < line.size() && std::isdigit(static_cast<unsigned char>(line[i + 1])));
        if (!start) {
            continue;
        }
        if ((line[i] == '-' || line[i] == '+') &&
            (i + 1 >= line.size() ||
             (!std::isdigit(static_cast<unsigned char>(line[i + 1])) && line[i + 1] != '.'))) {
            continue;
        }

        char *endp = nullptr;
        double value = strtod(line.c_str() + i, &endp);
        if (!endp || endp == line.c_str() + i) {
            continue;
        }

        size_t token_len = static_cast<size_t>(endp - (line.c_str() + i));
        std::string token(line.c_str() + i, token_len);
        size_t unit_pos = static_cast<size_t>(endp - line.c_str());
        while (unit_pos < line.size() && std::isspace(static_cast<unsigned char>(line[unit_pos]))) {
            ++unit_pos;
        }
        std::string unit;
        while (unit_pos < line.size() && std::isalpha(static_cast<unsigned char>(line[unit_pos])) && unit.size() < 2) {
            unit.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(line[unit_pos]))));
            ++unit_pos;
        }

        double mm = value;
        int score = 1;
        if (unit == "m") {
            mm = value * 1000.0;
            score = 3;
        } else if (unit == "cm") {
            mm = value * 10.0;
            score = 3;
        } else if (unit == "mm") {
            mm = value;
            score = 3;
        } else if (token.find('.') != std::string::npos && std::fabs(value) < 100.0) {
            mm = value * 1000.0;
            score = 2;
        }

        if (laser_accept_mm(mm) && score >= best_score) {
            last_mm = static_cast<int32_t>(std::lround(mm));
            best_score = score;
            found = true;
        }
        i = static_cast<size_t>(endp - line.c_str());
    }

    if (found) {
        result.found = true;
        result.distance_mm = last_mm;
        result.source = "ascii";
    }
    return found;
}

static LaserParseResult laser_parse_measurement(const std::vector<uint8_t> &bytes)
{
    LaserParseResult result;

    for (size_t i = 0; i + 7 < bytes.size(); ++i) {
        if (bytes[i] != 0xB4 || bytes[i + 1] != 0x69 || bytes[i + 2] != 0x04) {
            continue;
        }
        uint8_t xor_sum = 0;
        for (size_t j = i; j < i + 7; ++j) {
            xor_sum ^= bytes[j];
        }
        if (xor_sum != bytes[i + 7]) {
            continue;
        }
        uint32_t be = (static_cast<uint32_t>(bytes[i + 3]) << 24) |
                      (static_cast<uint32_t>(bytes[i + 4]) << 16) |
                      (static_cast<uint32_t>(bytes[i + 5]) << 8) |
                      static_cast<uint32_t>(bytes[i + 6]);
        if (laser_accept_mm(static_cast<double>(be))) {
            result.found = true;
            result.distance_mm = static_cast<int32_t>(be);
            result.source = "hex_b469";
        }
    }
    if (result.found) {
        return result;
    }

    std::string text = laser_ascii_from_bytes(bytes);
    size_t start = 0;
    while (start <= text.size()) {
        size_t end = text.find_first_of("\r\n", start);
        std::string line = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
        laser_parse_ascii_line(line, result);
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return result;
}

// The caller must own s_laser_mutex.  UART2 is also serviced by the runtime
// action task, so deleting/reinstalling the driver without this lock can leave
// uart_read_bytes() using a freed queue and trigger an interrupt watchdog.
static bool laser_exchange_locked(const char *cmd, int listen_ms)
{
    uart_flush_input(LASER_UART_NUM);
    std::string tx_text = std::string(cmd) + "\r\n";
    uart_write_bytes(LASER_UART_NUM, tx_text.data(), tx_text.size());
    uint8_t buf[256];
    std::string raw;
    std::vector<uint8_t> raw_bytes;
    int64_t end = esp_timer_get_time() + static_cast<int64_t>(listen_ms) * 1000;
    while (esp_timer_get_time() < end) {
        int n = uart_read_bytes(LASER_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(80));
        if (n > 0) {
            raw_bytes.insert(raw_bytes.end(), buf, buf + n);
            raw += printable_raw(buf, n);
            printf("[LASER RAW] %s\n", printable_raw(buf, n).c_str());
        }
    }
    if (raw.empty()) {
        return false;
    }
    LaserParseResult parsed = laser_parse_measurement(raw_bytes);
    if (parsed.found) {
        info("laser_parse", "distance_mm=" + std::to_string(parsed.distance_mm) + " source=" + parsed.source);
    }
    return true;
}

static bool cmd_test_laser_once()
{
    struct Candidate {
        gpio_num_t tx;
        gpio_num_t rx;
        int baud;
        const char *label;
    };
    const Candidate candidates[] = {
        {PIN_LASER_TX, PIN_LASER_RX, LASER_BAUD, "configured"},
        {PIN_LASER_RX, PIN_LASER_TX, LASER_BAUD, "swapped"},
        {PIN_LASER_TX, PIN_LASER_RX, 9600, "configured_9600"},
        {PIN_LASER_RX, PIN_LASER_TX, 9600, "swapped_9600"},
        {PIN_LASER_TX, PIN_LASER_RX, 115200, "configured_115200"},
        {PIN_LASER_RX, PIN_LASER_TX, 115200, "swapped_115200"},
        {PIN_LASER_TX, PIN_LASER_RX, 19200, "configured_19200"},
        {PIN_LASER_RX, PIN_LASER_TX, 19200, "swapped_19200"},
        {PIN_LASER_TX, PIN_LASER_RX, 57600, "configured_57600"},
        {PIN_LASER_RX, PIN_LASER_TX, 57600, "swapped_57600"},
    };

    // Keep the runtime owner idle for the complete probe.  The mutex below is
    // deliberately held across reconfiguration + reads for each candidate.
    s_laser_active_requested.store(false, std::memory_order_release);
    s_laser_continuous_active.store(false, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    vTaskDelay(pdMS_TO_TICKS(100));
    bool any_uart_response = false;
    gpio_num_t response_tx = PIN_LASER_TX;
    gpio_num_t response_rx = PIN_LASER_RX;
    int response_baud = LASER_BAUD;
    for (const auto &c : candidates) {
        char detail[96];
        snprintf(detail, sizeof(detail), "try %s TX=%d RX=%d baud=%d", c.label, c.tx, c.rx, c.baud);
        info("test_laser_once", detail);
        if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(2500)) != pdTRUE) {
            fail("test_laser_once", "laser UART mutex timeout");
            return false;
        }
        esp_err_t uart_err = uart_reconfigure(LASER_UART_NUM, c.tx, c.rx, c.baud, &s_uart_laser_ready);
        bool ok_get3 = false;
        bool ok_get4 = false;
        bool ok_sm = false;
        if (uart_err == ESP_OK) {
            snprintf(detail, sizeof(detail), "line levels before command: MCU_TX(GPIO%d)=%d MCU_RX(GPIO%d)=%d",
                     c.tx, gpio_get_level(c.tx), c.rx, gpio_get_level(c.rx));
            info("laser_lines", detail);
            ok_get3 = laser_exchange_locked("iGET:3", 500);
            ok_get4 = laser_exchange_locked("iGET:4", 500);
            ok_sm = laser_exchange_locked("iSM", 2500);
            // Always leave the emitter stopped after a diagnostic candidate.
            const char *halt = "iHALT\r\n";
            const char *off = "iLD:0\r\n";
            uart_write_bytes(LASER_UART_NUM, halt, strlen(halt));
            uart_write_bytes(LASER_UART_NUM, off, strlen(off));
            uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(200));
        }
        xSemaphoreGive(s_laser_mutex);
        if (ok_get3 || ok_get4 || ok_sm) {
            any_uart_response = true;
            response_tx = c.tx;
            response_rx = c.rx;
            response_baud = c.baud;
            s_laser_active_tx = c.tx;
            s_laser_active_rx = c.rx;
            s_laser_active_baud = c.baud;
        }
        if (ok_sm) {
            snprintf(detail, sizeof(detail), "raw response OK; active TX=%d RX=%d baud=%d iGET:3=%s iGET:4=%s",
                     c.tx, c.rx, c.baud, ok_get3 ? "OK" : "NO", ok_get4 ? "OK" : "NO");
            pass("test_laser_once", detail);
            return true;
        }
    }

    if (any_uart_response) {
        char detail[128];
        snprintf(detail, sizeof(detail),
                 "UART bytes received but no iSM distance; TX=%d RX=%d baud=%d; check command protocol",
                 response_tx, response_rx, response_baud);
        fail("test_laser_once", detail);
        return false;
    }
    fail("test_laser_once", "no response on configured/swapped pins or common baud rates; check 5V, GND, TX/RX, module protocol");
    return false;
}

static bool dashboard_laser_stop_continuous(bool reconfigure = true);

static bool cmd_stop_laser()
{
    s_laser_active_requested.store(false, std::memory_order_release);
    s_laser_measure_request.store(false, std::memory_order_release);
    s_laser_single_request.store(false, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    // An operator stop must also leave any page that assumes continuous laser
    // ownership. Otherwise the page and backend disagree and later presses
    // appear unresponsive until the user manually leaves and re-enters.
    device_ui_show_menu();

    // The action task owns the laser UART. Never delete/reinstall the UART
    // driver from the console concurrently with its polling loop.
    const int64_t deadline_us = esp_timer_get_time() + 2500000;
    while (s_laser_continuous_active.load(std::memory_order_acquire) &&
           esp_timer_get_time() < deadline_us) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    bool ok = !s_laser_continuous_active.load(std::memory_order_acquire);
    if (!ok) {
        // This fallback uses the same laser mutex as the action task, so UART
        // reconfiguration remains serialized even if the worker is delayed.
        ok = dashboard_laser_stop_continuous();
    }
    ok ? pass("stop_laser", "action task sent iHALT + iLD:0; UI returned to menu")
       : fail("stop_laser", "laser stop did not complete within timeout");
    return ok;
}

static bool cmd_test_laser_cont()
{
    s_laser_active_requested.store(false, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(2500)) != pdTRUE) {
        fail("test_laser_cont", "laser UART mutex timeout");
        return false;
    }
    esp_err_t uart_err = uart_reconfigure(LASER_UART_NUM, s_laser_active_tx, s_laser_active_rx,
                                          s_laser_active_baud, &s_uart_laser_ready);
    if (uart_err != ESP_OK || !laser_exchange_locked("iACM", 500)) {
        xSemaphoreGive(s_laser_mutex);
        fail("test_laser_cont", "no response to iACM");
        return false;
    }
    uint8_t buf[256];
    bool got = false;
    int64_t end = esp_timer_get_time() + 8LL * 1000 * 1000;
    while (esp_timer_get_time() < end) {
        int n = uart_read_bytes(LASER_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (n > 0) {
            got = true;
            printf("[LASER RAW] %s\n", printable_raw(buf, n).c_str());
        }
    }
    const char *halt = "iHALT\r\n";
    const char *off = "iLD:0\r\n";
    uart_write_bytes(LASER_UART_NUM, halt, strlen(halt));
    uart_write_bytes(LASER_UART_NUM, off, strlen(off));
    uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(200));
    xSemaphoreGive(s_laser_mutex);
    got ? pass("test_laser_cont", "continuous raw data received then iHALT sent")
        : fail("test_laser_cont", "continuous mode started but no stream received");
    return got;
}

static void dashboard_bno_update_accel(const std::vector<uint8_t> &p, size_t off)
{
    dashboard_lock();
    s_dash.t_us = esp_timer_get_time();
    s_dash.bno_ready = true;
    s_dash.ax = q_to_float(bno_s16(p, off + 4), 8);
    s_dash.ay = q_to_float(bno_s16(p, off + 6), 8);
    s_dash.az = q_to_float(bno_s16(p, off + 8), 8);
    s_dash.body_ax = -s_dash.az;
    s_dash.body_ay = -s_dash.ay;
    s_dash.body_az = -s_dash.ax;
    s_dash.bno_status = p[off + 2] & 0x03;
    ++s_dash.bno_count;
    dashboard_unlock();
}

static void dashboard_bno_update_gyro(const std::vector<uint8_t> &p, size_t off)
{
    const int64_t sample_us = esp_timer_get_time();
    dashboard_lock();
    s_dash.t_us = sample_us;
    s_dash.bno_ready = true;
    s_dash.gx = q_to_float(bno_s16(p, off + 4), 9);
    s_dash.gy = q_to_float(bno_s16(p, off + 6), 9);
    s_dash.gz = q_to_float(bno_s16(p, off + 8), 9);
    s_dash.body_gx = -s_dash.gz;
    s_dash.body_gy = -s_dash.gy;
    s_dash.body_gz = -s_dash.gx;
    if (s_dash.p2p_motion_start_us > 0) {
        const float gyro_dps = std::sqrt(s_dash.body_gx * s_dash.body_gx +
                                         s_dash.body_gy * s_dash.body_gy +
                                         s_dash.body_gz * s_dash.body_gz) * 57.2957795f;
        s_dash.p2p_max_gyro_dps = std::max(s_dash.p2p_max_gyro_dps, gyro_dps);
    }
    s_dash.bno_status = p[off + 2] & 0x03;
    ++s_dash.bno_count;
    const float gx = s_dash.gx;
    const float gy = s_dash.gy;
    const float gz = s_dash.gz;
    dashboard_unlock();
    s_fusion.updateGyroscope(sample_us, gx, gy, gz);
}

static void dashboard_bno_update_mag(const std::vector<uint8_t> &p, size_t off)
{
    dashboard_lock();
    s_dash.t_us = esp_timer_get_time();
    s_dash.bno_ready = true;
    s_dash.mx = q_to_float(bno_s16(p, off + 4), 4);
    s_dash.my = q_to_float(bno_s16(p, off + 6), 4);
    s_dash.mz = q_to_float(bno_s16(p, off + 8), 4);
    s_dash.bno_status = p[off + 2] & 0x03;
    ++s_dash.bno_count;
    dashboard_unlock();
}

static void dashboard_bno_update_linear_accel(const std::vector<uint8_t> &p, size_t off)
{
    const float x = q_to_float(bno_s16(p, off + 4), 8);
    const float y = q_to_float(bno_s16(p, off + 6), 8);
    const float z = q_to_float(bno_s16(p, off + 8), 8);
    const float magnitude = std::sqrt(x * x + y * y + z * z);
    dashboard_lock();
    if (s_dash.p2p_motion_start_us > 0) {
        s_dash.p2p_max_linear_accel_mps2 = std::max(s_dash.p2p_max_linear_accel_mps2, magnitude);
        s_dash.p2p_accel_deviation_sum += magnitude;
        ++s_dash.p2p_accel_sample_count;
    }
    if (s_room.active) {
        s_room.scan_max_linear_accel_mps2 =
            std::max(s_room.scan_max_linear_accel_mps2, magnitude);
        // BNO086 linear acceleration has gravity removed.  This cannot detect
        // slow constant-velocity translation, so it is a warning only and is
        // never used to discard a range sample.
        s_room.scan_motion_risk = s_room.scan_max_linear_accel_mps2 > 1.50f ? 2 :
                                  s_room.scan_max_linear_accel_mps2 > 0.50f ? 1 : 0;
    }
    dashboard_unlock();
}

static void dashboard_bno_update_quat(const std::vector<uint8_t> &p, size_t off)
{
    dashboard_lock();
    s_dash.t_us = esp_timer_get_time();
    s_dash.bno_ready = true;
    s_dash.qi = q_to_float(bno_s16(p, off + 4), 14);
    s_dash.qj = q_to_float(bno_s16(p, off + 6), 14);
    s_dash.qk = q_to_float(bno_s16(p, off + 8), 14);
    s_dash.qr = q_to_float(bno_s16(p, off + 10), 14);
    s_dash.bno_status = p[off + 2] & 0x03;
    ++s_dash.bno_count;
    DashboardState snap = s_dash;
    dashboard_unlock();

    s_fusion.updateAttitude(snap.t_us, snap.qi, snap.qj, snap.qk, snap.qr,
                            snap.ax, snap.ay, snap.az, snap.gx, snap.gy, snap.gz,
                            snap.bno_status);
    FusionState fused = s_fusion.snapshot();
    FusionPose body_pose = s_fusion.capturePose();
    dashboard_lock();
    s_dash.fusion_yaw = fused.yaw; s_dash.fusion_pitch = fused.pitch; s_dash.fusion_roll = fused.roll;
    s_dash.fusion_confidence = fused.confidence;
    s_dash.body_qi = body_pose.qi; s_dash.body_qj = body_pose.qj;
    s_dash.body_qk = body_pose.qk; s_dash.body_qr = body_pose.qr;
    dashboard_unlock();
}

static void dashboard_bno_update_game_quat(const std::vector<uint8_t> &p, size_t off)
{
    TimedQuaternion sample;
    sample.t_us = esp_timer_get_time();
    sample.qi = q_to_float(bno_s16(p, off + 4), 14);
    sample.qj = q_to_float(bno_s16(p, off + 6), 14);
    sample.qk = q_to_float(bno_s16(p, off + 8), 14);
    sample.qr = q_to_float(bno_s16(p, off + 10), 14);
    sample.accuracy = p[off + 2] & 0x03;
    taskENTER_CRITICAL(&s_game_rv_mux);
    s_game_rv_history[s_game_rv_head] = sample;
    s_game_rv_head = (s_game_rv_head + 1) % P2P_IMU_HISTORY_CAPACITY;
    s_game_rv_count = std::min(s_game_rv_count + 1, P2P_IMU_HISTORY_CAPACITY);
    taskEXIT_CRITICAL(&s_game_rv_mux);
    dashboard_lock();
    ++s_dash.game_rv_count;
    dashboard_unlock();
}

static size_t dashboard_bno_process_motion(const std::vector<uint8_t> &p, size_t off)
{
    uint8_t rid = p[off];
    size_t remain = p.size() - off;
    if ((rid == BNO_RPT_ACCEL || rid == BNO_RPT_GYRO || rid == BNO_RPT_MAG ||
         rid == BNO_RPT_LINEAR_ACCEL) && remain >= 10) {
        if (rid == BNO_RPT_ACCEL) {
            dashboard_bno_update_accel(p, off);
        } else if (rid == BNO_RPT_GYRO) {
            dashboard_bno_update_gyro(p, off);
        } else if (rid == BNO_RPT_LINEAR_ACCEL) {
            dashboard_bno_update_linear_accel(p, off);
        } else {
            dashboard_bno_update_mag(p, off);
        }
        return 10;
    }
    if (rid == BNO_RPT_ROTATION_VECTOR && remain >= 14) {
        // This is the BNO086 nine-axis solution: gyro integration is
        // stabilised by gravity and magnetic north.  It is the sole attitude
        // source for the level display and P2P ray projection.
        dashboard_bno_update_quat(p, off);
        return 14;
    }
    if (rid == BNO_RPT_GAME_RV && remain >= 12) {
        // Keep Game RV in a separate timestamped history.  P2P precision
        // mode uses only this non-magnetic stream for both A and B, while the
        // magnetic rotation vector remains the UI/world-heading source.
        dashboard_bno_update_game_quat(p, off);
        return 12;
    }
    return 0;
}

static void dashboard_bno_process_packet(const BnoPacket &pkt)
{
    if (pkt.payload.empty()) {
        return;
    }
    if (pkt.channel == BNO_CH_INPUT || pkt.channel == BNO_CH_WAKE_INPUT || pkt.channel == BNO_CH_GYRO_RV) {
        size_t off = 0;
        while (off < pkt.payload.size()) {
            if (pkt.payload[off] == BNO_RPT_TIMEBASE) {
                if (off + 5 > pkt.payload.size()) {
                    break;
                }
                off += 5;
                continue;
            }
            size_t used = dashboard_bno_process_motion(pkt.payload, off);
            if (used == 0) {
                break;
            }
            off += used;
        }
    }
}

static void dashboard_bno_log_snapshot()
{
    DashboardState snap;
    dashboard_lock();
    snap = s_dash;
    dashboard_unlock();
    if (!snap.bno_ready) {
        return;
    }
    char line[360];
    snprintf(line, sizeof(line),
             "%lld,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%u\n",
             static_cast<long long>(snap.t_us),
             static_cast<double>(snap.ax), static_cast<double>(snap.ay), static_cast<double>(snap.az),
             static_cast<double>(snap.gx), static_cast<double>(snap.gy), static_cast<double>(snap.gz),
             static_cast<double>(snap.qi), static_cast<double>(snap.qj), static_cast<double>(snap.qk), static_cast<double>(snap.qr),
             static_cast<double>(snap.mx), static_cast<double>(snap.my), static_cast<double>(snap.mz),
             snap.bno_status);
    dashboard_log_csv("bno086.csv", line);
}

static bool ensure_dataset_capture_directory()
{
    const std::string path = std::string(SD_MOUNT_POINT) + DATASET_CAPTURE_DIR;
    struct stat st = {};
    if (stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    return mkdir(path.c_str(), 0775) == 0 || errno == EEXIST;
}

static esp_err_t camera_http_init(bool fast_restore = false);

static bool dashboard_capture_photo(std::string *out_path, const char *name_prefix = "photo",
                                    bool high_resolution = false)
{
    const int64_t timing_begin = esp_timer_get_time();
    int64_t timing_still_ready = timing_begin;
    int64_t timing_settled = timing_begin;
    int64_t timing_frame_ready = timing_begin;
    int64_t timing_written = timing_begin;
    if (!s_camera_http_ready || !s_camera_mutex) {
        dashboard_set_error("camera not ready");
        return false;
    }
    if (!s_sd_mounted) {
        dashboard_set_error("SD not mounted; photo not saved");
        return false;
    }
    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(4000)) != pdTRUE) {
        dashboard_set_error("camera busy");
        return false;
    }

    bool dedicated_still_mode = false;
    bool i2c_bus_owned = false;
    if (high_resolution) {
        // esp-camera owns a persistent SCCB device handle on the application's
        // shared I2C bus.  Serialize its removal/recreation against BNO086,
        // touch and PCA temporary devices or IDF can reject the removal and
        // leak SCCB handles after repeated photos.
        if (!i2c_take_bus(2000)) {
            xSemaphoreGive(s_camera_mutex);
            dashboard_set_error("shared I2C busy; cannot enter still mode");
            return false;
        }
        i2c_bus_owned = true;
        // esp32-camera fixes its DMA dimensions when esp_camera_init() runs.
        // Reprogramming only OV5640 framesize registers leaves the active DMA
        // stream at the old VGA timing and yields NO-SOI/NO-EOI frames.  Stop
        // preview completely and build a one-shot 5 MP driver instead.
        s_camera_http_ready = false;
        esp_camera_return_all();
        esp_camera_deinit();
        camera_config_t still = camera_jpeg_config(CAMERA_STILL_FRAME_SIZE, 8, 1,
                                                   CAMERA_GRAB_WHEN_EMPTY);
        esp_err_t still_err = camera_use_shared_i2c(still);
        if (still_err == ESP_OK) still_err = esp_camera_init(&still);
        if (still_err != ESP_OK) {
            esp_camera_deinit();
            const esp_err_t restore_err = camera_http_init(true);
            if (i2c_bus_owned) i2c_give_bus();
            xSemaphoreGive(s_camera_mutex);
            dashboard_set_error(std::string("camera still-mode init failed: ") +
                                esp_err_to_name(still_err) + "; preview restore=" +
                                esp_err_to_name(restore_err));
            return false;
        }
        dedicated_still_mode = true;
        timing_still_ready = esp_timer_get_time();
        // 注意:该广角模块为固定焦距镜头,无 AF 硬件;不要在此路径触发
        // esp_camera_af_*(固定焦距时 AF 固件加载会失败并空耗约 6 秒)。
        vTaskDelay(pdMS_TO_TICKS(120));
        // Let automatic exposure and white balance settle at the new timing.
        for (int i = 0; i < 2; ++i) {
            camera_fb_t *warmup = esp_camera_fb_get();
            if (warmup) esp_camera_fb_return(warmup);
        }
        timing_settled = esp_timer_get_time();
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        if (dedicated_still_mode) {
            esp_camera_deinit();
            camera_http_init(true);
        }
        if (i2c_bus_owned) i2c_give_bus();
        xSemaphoreGive(s_camera_mutex);
        dashboard_set_error("camera capture failed");
        return false;
    }
    timing_frame_ready = esp_timer_get_time();

    uint8_t *jpg = fb->buf;
    size_t jpg_len = fb->len;
    const size_t captured_width = fb->width;
    const size_t captured_height = fb->height;
    bool converted = false;
    if (fb->format != PIXFORMAT_JPEG) {
        jpg = nullptr;
        jpg_len = 0;
        converted = frame2jpg(fb, 85, &jpg, &jpg_len);
    }
    bool ok = false;
    const char *prefix = (name_prefix && name_prefix[0]) ? name_prefix : "photo";
    const std::string directory = high_resolution ? DATASET_CAPTURE_DIR : "";
    if (high_resolution && !ensure_dataset_capture_directory()) {
        if (converted && jpg) free(jpg);
        esp_camera_fb_return(fb);
        if (dedicated_still_mode) {
            esp_camera_deinit();
            camera_http_init(true);
        }
        if (i2c_bus_owned) i2c_give_bus();
        xSemaphoreGive(s_camera_mutex);
        dashboard_set_error("cannot create /dataset_capture on SD");
        return false;
    }
    std::string rel = directory + "/" + std::string(prefix) + "_" +
                      std::to_string(static_cast<long long>(esp_timer_get_time())) + ".jpg";
    std::string path = std::string(SD_MOUNT_POINT) + rel;
    if (jpg && jpg_len > 0) {
        FILE *f = fopen(path.c_str(), "wb");
        if (f) {
            ok = fwrite(jpg, 1, jpg_len, f) == jpg_len && fflush(f) == 0 && fsync(fileno(f)) == 0;
            fclose(f);
        }
    }
    timing_written = esp_timer_get_time();
    if (converted && jpg) {
        free(jpg);
    }
    esp_camera_fb_return(fb);
    esp_err_t restore_err = ESP_OK;
    if (dedicated_still_mode) {
        esp_camera_deinit();
        restore_err = camera_http_init(true);
        if (restore_err != ESP_OK) {
            ESP_LOGE("camera_capture", "VGA preview restore failed: %s", esp_err_to_name(restore_err));
        }
    }
    if (i2c_bus_owned) i2c_give_bus();
    xSemaphoreGive(s_camera_mutex);

    dashboard_lock();
    if (ok && restore_err == ESP_OK) {
        ++s_dash.photo_count;
        snprintf(s_dash.last_photo, sizeof(s_dash.last_photo), "%s", rel.c_str());
        s_dash.last_error[0] = '\0';
    } else if (!ok) {
        snprintf(s_dash.last_error, sizeof(s_dash.last_error), "photo write failed errno=%d", errno);
    } else {
        snprintf(s_dash.last_error, sizeof(s_dash.last_error), "photo saved but preview restore failed: %s",
                 esp_err_to_name(restore_err));
    }
    dashboard_unlock();
    if (ok) {
        dashboard_log_event(high_resolution ? "dataset_photo" : "photo", rel);
        ESP_LOGI("camera_capture", "saved %s (%ux%u, %u bytes)", rel.c_str(),
                 static_cast<unsigned>(captured_width),
                 static_cast<unsigned>(captured_height),
                 static_cast<unsigned>(jpg_len));
        if (out_path) {
            *out_path = rel;
        }
        // 照片加入上传队列(设备 → PC,供 AI 识别与几何提取)。SD 始终保留原件。
        pc_link_queue_upload(rel.c_str());
    }
    const int64_t timing_done = esp_timer_get_time();
    ESP_LOGI("camera_capture_timing",
             "total=%.0fms still_init=%.0fms settle=%.0fms capture=%.0fms sd_write=%.0fms preview_restore=%.0fms",
             static_cast<double>(timing_done - timing_begin) / 1000.0,
             static_cast<double>(timing_still_ready - timing_begin) / 1000.0,
             static_cast<double>(timing_settled - timing_still_ready) / 1000.0,
             static_cast<double>(timing_frame_ready - timing_settled) / 1000.0,
             static_cast<double>(timing_written - timing_frame_ready) / 1000.0,
             static_cast<double>(timing_done - timing_written) / 1000.0);
    return ok && restore_err == ESP_OK;
}

static void room_publish_locked()
{
    s_dash.room_active = s_room.active;
    s_dash.room_complete = s_room.complete;
    s_dash.room_session_id = s_room.session_id;
    s_dash.room_point_count = static_cast<uint16_t>(s_room.scan_valid_count + s_room.scan_invalid_count);
    s_dash.room_last_segment_m = s_room.last_segment_m;
    s_dash.room_open_perimeter_m = s_room.open_perimeter_m;
    s_dash.room_closure_m = s_room.closure_m;
    s_dash.room_area_xy_m2 = s_room.area_xy_m2;
    s_dash.room_current_angle_deg = s_room.scan_have_angle ?
        s_room.scan_unwrapped_deg - s_room.scan_start_azimuth_deg : 0.0f;
    s_dash.room_rotation_deg = std::fabs(s_dash.room_current_angle_deg);
    s_dash.room_pitch_deg = s_room.scan_current_pitch_deg;
    s_dash.room_valid_count = s_room.scan_valid_count;
    s_dash.room_invalid_count = s_room.scan_invalid_count;
    s_dash.room_pose_ready = s_room.scan_pose_ready;
    s_dash.room_motion_risk = s_room.scan_motion_risk;
    s_dash.room_max_linear_accel_mps2 = s_room.scan_max_linear_accel_mps2;
    s_dash.room_scan_coverage_complete = s_room.scan_coverage_complete;
    snprintf(s_dash.room_scan_file, sizeof(s_dash.room_scan_file), "%s", s_room.scan_file);
}

static float wrap_degrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

static float yaw_degrees_from_pose(const FusionPose &pose)
{
    return std::atan2(2.0f * (pose.qr * pose.qk - pose.qi * pose.qj),
                      1.0f - 2.0f * (pose.qi * pose.qi + pose.qk * pose.qk)) * 57.2957795f;
}

static void level_degrees_from_pose(const FusionPose &pose, float *roll_deg, float *pitch_deg)
{
    if (pitch_deg) {
        *pitch_deg = std::asin(std::clamp(
            2.0f * (pose.qr * pose.qi + pose.qj * pose.qk), -1.0f, 1.0f)) * 57.2957795f;
    }
    if (roll_deg) {
        *roll_deg = std::atan2(2.0f * (pose.qr * pose.qj - pose.qi * pose.qk),
            1.0f - 2.0f * (pose.qi * pose.qi + pose.qj * pose.qj)) * 57.2957795f;
    }
}

// Update the displayed/unwrapped scan angle from the 50 Hz Game Rotation
// Vector stream.  This deliberately runs independently of the rangefinder:
// weak targets may delay or suppress range reports, but they must not freeze
// the device angle shown to the operator.
static void room_scan_update_attitude()
{
    dashboard_lock();
    const bool active = s_room.active;
    const bool already_complete = s_room.scan_coverage_complete;
    dashboard_unlock();
    if (!active || already_complete) return;

    const int64_t now_us = esp_timer_get_time();
    FusionPose pose;
    uint16_t sync_error_ms = 0;
    const bool fresh = p2p_pose_at(now_us, &pose, &sync_error_ms) && sync_error_ms <= 100;
    float roll_deg = 0.0f, pitch_deg = 0.0f, azimuth_deg = 0.0f;
    if (fresh) {
        level_degrees_from_pose(pose, &roll_deg, &pitch_deg);
        azimuth_deg = yaw_degrees_from_pose(pose);
    }

    dashboard_lock();
    if (!s_room.active) {
        dashboard_unlock();
        return;
    }
    s_room.scan_pose_ready = fresh;
    if (fresh) {
        s_room.scan_current_pitch_deg = pitch_deg;
        if (!s_room.scan_have_angle) {
            s_room.scan_have_angle = true;
            s_room.scan_start_azimuth_deg = azimuth_deg;
            s_room.scan_last_azimuth_deg = azimuth_deg;
            s_room.scan_unwrapped_deg = azimuth_deg;
            s_room.scan_last_saved_angle_deg = -10.0f;
        } else {
            s_room.scan_unwrapped_deg +=
                wrap_degrees(azimuth_deg - s_room.scan_last_azimuth_deg);
            s_room.scan_last_azimuth_deg = azimuth_deg;
        }
        s_room.scan_coverage_complete =
            std::fabs(s_room.scan_unwrapped_deg - s_room.scan_start_azimuth_deg) >= 355.0f;
    }
    room_publish_locked();
    dashboard_unlock();
}

static void room_scan_close_file()
{
    if (!s_sd_log_mutex) return;
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return;
    if (s_room_scan_file) {
        fflush(s_room_scan_file);
        fsync(fileno(s_room_scan_file));
        fclose(s_room_scan_file);
        s_room_scan_file = nullptr;
    }
    xSemaphoreGive(s_sd_log_mutex);
}

static bool room_scan_open_file(uint32_t session_id, float start_roll_deg, float start_pitch_deg,
                                char *out_file, size_t out_size)
{
    if (!s_sd_mounted || !s_sd_log_mutex || !out_file || out_size == 0) return false;
    snprintf(out_file, out_size, "/scan_%06lu.csv", static_cast<unsigned long>(session_id));
    const std::string path = std::string(SD_MOUNT_POINT) + out_file;
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return false;
    s_room_scan_file = fopen(path.c_str(), "wb");
    bool ok = s_room_scan_file != nullptr;
    if (ok) {
        setvbuf(s_room_scan_file, nullptr, _IOFBF, 8192);
        ok = fprintf(s_room_scan_file,
            "# schema=laser_room_scan_v2\n"
            "# session_id=%lu\n"
            "# angle_source=bno086_game_rotation_vector_50hz\n"
            "# range_mode=iFACM_20hz\n"
            "# projection=full_quaternion_fixed_station\n"
            "# attitude_columns=absolute_world_degrees\n"
            "# translation_policy=warning_only_unobservable_without_external_position\n"
            "# laser_offset_mm=%.3f,%.3f,%.3f\n"
            "# laser_direction=%.6f,%.6f,%.6f\n"
            "# projection_origin=tripod_reference\n"
            "# level_gate=disabled_pitch_and_roll_are_projected\n"
            "# start_roll_deg=%.4f\n"
            "# start_pitch_deg=%.4f\n"
            "timestamp_us,angle_deg,distance_mm,valid,roll_deg,pitch_deg,quality,sync_error_ms,discontinuity,x_mm,y_mm,z_mm,horizontal_distance_mm,motion_risk\n",
            static_cast<unsigned long>(session_id),
             static_cast<double>(FUSION_PIVOT_OFFSET_X_M * 1000.0f),
             static_cast<double>(FUSION_PIVOT_OFFSET_Y_M * 1000.0f),
             static_cast<double>(FUSION_PIVOT_OFFSET_Z_M * 1000.0f),
            static_cast<double>(FUSION_LASER_DIR_X),
            static_cast<double>(FUSION_LASER_DIR_Y),
            static_cast<double>(FUSION_LASER_DIR_Z),
            static_cast<double>(start_roll_deg),
            static_cast<double>(start_pitch_deg)) > 0;
        if (ok) ok = fflush(s_room_scan_file) == 0;
    }
    if (!ok && s_room_scan_file) {
        fclose(s_room_scan_file);
        s_room_scan_file = nullptr;
    }
    xSemaphoreGive(s_sd_log_mutex);
    return ok;
}

static bool room_scan_append_sample()
{
    int32_t distance_mm = -1;
    int64_t sample_us = 0;
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, 0) != pdTRUE) return false;
    distance_mm = s_laser_latest_mm;
    sample_us = s_laser_latest_us;
    xSemaphoreGive(s_laser_mutex);
    if (sample_us <= 0) return false;

    dashboard_lock();
    const bool active = s_room.active;
    const bool coverage_complete = s_room.scan_coverage_complete;
    const int64_t previous_laser_us = s_room.scan_last_laser_us;
    dashboard_unlock();
    if (!active || coverage_complete || sample_us == previous_laser_us) return false;

    FusionPose pose;
    uint16_t sync_error_ms = 0;
    const bool synced = p2p_pose_at(sample_us, &pose, &sync_error_ms);
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    if (synced) level_degrees_from_pose(pose, &roll_deg, &pitch_deg);
    float azimuth_deg = synced ? yaw_degrees_from_pose(pose) : 0.0f;

    dashboard_lock();
    if (!s_room.active || s_room.scan_last_laser_us == sample_us) {
        dashboard_unlock();
        return false;
    }
    s_room.scan_last_laser_us = sample_us;
    // Map the timestamp-matched range pose onto the nearest branch of the
    // continuously unwrapped attitude.  Sparse laser frames can therefore no
    // longer alias a rotation by more than 180 degrees.
    const float sample_unwrapped_deg = synced && s_room.scan_have_angle ?
        s_room.scan_unwrapped_deg + wrap_degrees(azimuth_deg - s_room.scan_last_azimuth_deg) :
        s_room.scan_unwrapped_deg;
    const float relative_angle_deg = s_room.scan_have_angle ?
        sample_unwrapped_deg - s_room.scan_start_azimuth_deg : 0.0f;
    const bool angle_spacing_ok = !s_room.scan_have_angle ||
        std::fabs(relative_angle_deg - s_room.scan_last_saved_angle_deg) >= 0.20f;
    const bool interval_ok = sample_us - s_room.scan_last_saved_us >= 40000;
    const bool distance_ok = distance_mm > 50 && distance_mm < 20000;
    float x_m = 0.0f, y_m = 0.0f, z_m = 0.0f;
    const bool projected = synced && distance_ok &&
        s_fusion.projectLaserPoint(pose, distance_mm, &x_m, &y_m, &z_m);
    const bool valid = synced && sync_error_ms <= 100 && distance_ok && projected;
    if (!angle_spacing_ok || !interval_ok) {
        room_publish_locked();
        dashboard_unlock();
        return false;
    }
    s_room.scan_last_saved_us = sample_us;
    s_room.scan_last_saved_angle_deg = relative_angle_deg;
    const bool discontinuity = s_room.scan_last_distance_mm > 0 && distance_mm > 0 &&
        std::abs(distance_mm - s_room.scan_last_distance_mm) >
            std::max(250, static_cast<int>(0.20f * std::min(distance_mm, s_room.scan_last_distance_mm)));
    s_room.scan_last_distance_mm = distance_mm;
    s_room.scan_pose_ready = synced;
    if (valid) ++s_room.scan_valid_count; else ++s_room.scan_invalid_count;
    const uint32_t index = static_cast<uint32_t>(s_room.scan_valid_count) + s_room.scan_invalid_count;
    const uint8_t motion_risk = s_room.scan_motion_risk;
    room_publish_locked();
    dashboard_unlock();

    if (!s_sd_log_mutex || xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
    bool written = s_room_scan_file && fprintf(s_room_scan_file,
        "%lld,%.4f,%ld,%u,%.4f,%.4f,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%u\n",
        static_cast<long long>(sample_us), static_cast<double>(relative_angle_deg),
        static_cast<long>(distance_mm), valid ? 1u : 0u,
        static_cast<double>(roll_deg), static_cast<double>(pitch_deg),
        valid ? 3u : 0u, static_cast<unsigned>(sync_error_ms), discontinuity ? 1u : 0u,
        static_cast<double>(valid ? x_m * 1000.0f : 0.0f),
        static_cast<double>(valid ? y_m * 1000.0f : 0.0f),
        static_cast<double>(valid ? z_m * 1000.0f : 0.0f),
        static_cast<double>(valid ? hypotf(x_m, y_m) * 1000.0f : 0.0f),
        static_cast<unsigned>(motion_risk)) > 0;
    if (written && index % 20u == 0u) written = fflush(s_room_scan_file) == 0;
    xSemaphoreGive(s_sd_log_mutex);
    if (!written) dashboard_set_error("scan.csv SD write failed");
    return written;
}

static void room_recalculate_locked()
{
    s_room.last_segment_m = 0.0f;
    s_room.open_perimeter_m = 0.0f;
    s_room.closure_m = 0.0f;
    s_room.area_xy_m2 = 0.0f;
    for (uint16_t i = 1; i < s_room.count; ++i) {
        const float dx = s_room.x[i] - s_room.x[i - 1];
        const float dy = s_room.y[i] - s_room.y[i - 1];
        const float dz = s_room.z[i] - s_room.z[i - 1];
        const float segment = std::sqrt(dx * dx + dy * dy + dz * dz);
        s_room.open_perimeter_m += segment;
        s_room.last_segment_m = segment;
    }
    if (s_room.count >= 2) {
        const uint16_t last = s_room.count - 1;
        const float dx = s_room.x[last] - s_room.x[0];
        const float dy = s_room.y[last] - s_room.y[0];
        const float dz = s_room.z[last] - s_room.z[0];
        s_room.closure_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    if (s_room.count >= 3) {
        double twice_area = 0.0;
        for (uint16_t i = 0; i < s_room.count; ++i) {
            const uint16_t j = static_cast<uint16_t>((i + 1) % s_room.count);
            twice_area += static_cast<double>(s_room.x[i]) * s_room.y[j] -
                          static_cast<double>(s_room.x[j]) * s_room.y[i];
        }
        s_room.area_xy_m2 = static_cast<float>(std::fabs(twice_area) * 0.5);
    }
    room_publish_locked();
}

static bool room_survey_begin(bool moving_mode)
{
    const uint32_t session_id = room_allocate_session_id();
    if (session_id == 0) {
        dashboard_set_error("room session NVS allocation failed");
        return false;
    }
    room_scan_close_file();
    FusionPose start_pose;
    uint16_t start_sync_ms = 0;
    const bool attitude_ready = p2p_pose_at(esp_timer_get_time(), &start_pose, &start_sync_ms);
    float roll_deg = 0.0f, pitch_deg = 0.0f;
    if (attitude_ready) level_degrees_from_pose(start_pose, &roll_deg, &pitch_deg);
    if (!s_sd_mounted || !attitude_ready || start_sync_ms > 100) {
        dashboard_set_error(!s_sd_mounted ? "scan requires SD card" :
                            "scan requires fresh Game RV attitude");
        return false;
    }
    char scan_file[48] = "";
    if (!room_scan_open_file(session_id, roll_deg, pitch_deg, scan_file, sizeof(scan_file))) {
        dashboard_set_error("cannot create scan.csv on SD");
        return false;
    }
    s_fusion.resetPose();
    s_fusion.resetMeasurementPair();
    dashboard_lock();
    s_room = {};
    s_room.active = true;
    s_room.session_id = session_id;
    snprintf(s_room.scan_file, sizeof(s_room.scan_file), "%s", scan_file);
    s_room.scan_start_roll_deg = roll_deg;
    s_room.scan_start_pitch_deg = pitch_deg;
    s_room.scan_current_pitch_deg = pitch_deg;
    s_room.scan_pose_ready = true;
    s_room.scan_have_angle = true;
    s_room.scan_start_azimuth_deg = yaw_degrees_from_pose(start_pose);
    s_room.scan_last_azimuth_deg = s_room.scan_start_azimuth_deg;
    s_room.scan_unwrapped_deg = s_room.scan_start_azimuth_deg;
    s_room.scan_last_saved_angle_deg = -10.0f;
    // With the fixed IMU origin, room surveying is valid only
    // from one fixed station.  Keep the callback parameter for UI ABI safety.
    s_room_moving_mode = false;
    s_dash.measure_session_id = 0;
    s_dash.measure_point_count = 0;
    s_dash.point_distance_m = -1.0f;
    s_dash.room_dxf_saved = false;
    s_dash.room_dxf_file[0] = '\0';
    room_publish_locked();
    dashboard_unlock();
    s_room_upload_state.store(RoomUploadState::NONE, std::memory_order_release);
    dashboard_log_event("room", "start id=" + std::to_string(session_id) +
                                " mode=fixed_station_ifacm_20hz angle_source=game_rv_50hz file=" + scan_file);
    s_laser_active_requested.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    return true;
}

static bool room_survey_append_point(int64_t t_us, int32_t laser_mm, float x, float y, float z)
{
    uint32_t session_id = 0;
    uint16_t point_index = 0;
    float segment = 0.0f, perimeter = 0.0f, closure = 0.0f, area = 0.0f;
    dashboard_lock();
    const bool was_active = s_room.active;
    if (!was_active || s_room.count >= ROOM_MAX_POINTS) {
        dashboard_unlock();
        if (was_active) dashboard_set_error("room point limit reached");
        return false;
    }
    point_index = s_room.count;
    s_room.x[point_index] = x;
    s_room.y[point_index] = y;
    s_room.z[point_index] = z;
    ++s_room.count;
    room_recalculate_locked();
    session_id = s_room.session_id;
    segment = s_room.last_segment_m;
    perimeter = s_room.open_perimeter_m;
    closure = s_room.closure_m;
    area = s_room.area_xy_m2;
    dashboard_unlock();

    char line[320];
    snprintf(line, sizeof(line), "%lld,%lu,%u,%ld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
             static_cast<long long>(t_us), static_cast<unsigned long>(session_id), point_index + 1,
             static_cast<long>(laser_mm), static_cast<double>(x), static_cast<double>(y), static_cast<double>(z),
             static_cast<double>(segment), static_cast<double>(perimeter),
             static_cast<double>(closure), static_cast<double>(area));
    const bool saved = dashboard_log_csv("room_points.csv", line);
    dashboard_log_event("room", "point id=" + std::to_string(session_id) +
                                 " index=" + std::to_string(point_index + 1));
    if (!saved) dashboard_set_error("room point SD write failed");
    return saved;
}

static bool room_survey_undo()
{
    uint32_t session_id = 0;
    uint16_t removed_index = 0;
    dashboard_lock();
    if (!s_room.active || s_room.count == 0) {
        dashboard_unlock();
        dashboard_set_error("room has no point to undo");
        return false;
    }
    session_id = s_room.session_id;
    removed_index = s_room.count;
    --s_room.count;
    room_recalculate_locked();
    s_dash.last_error[0] = '\0';
    dashboard_unlock();
    dashboard_log_event("room", "undo id=" + std::to_string(session_id) +
                                 " point=" + std::to_string(removed_index));
    return true;
}

static bool room_write_dxf(const RoomSurveyState &room, char *out_file, size_t out_file_size)
{
    if (!s_sd_mounted || !s_sd_log_mutex || room.count < 3 || !out_file || out_file_size == 0) return false;
    bool has_closure_sample = false;
    if (room.count >= 4) {
        const float average_segment = room.open_perimeter_m / static_cast<float>(room.count - 1);
        const float duplicate_limit = std::clamp(average_segment * 0.25f, 0.10f, 0.50f);
        has_closure_sample = room.closure_m <= duplicate_limit;
    }
    const uint16_t vertex_count = has_closure_sample ? room.count - 1 : room.count;
    if (vertex_count < 3) return false;

    std::array<float, ROOM_MAX_POINTS> adjusted_x = {};
    std::array<float, ROOM_MAX_POINTS> adjusted_y = {};
    for (uint16_t i = 0; i < vertex_count; ++i) {
        adjusted_x[i] = room.x[i];
        adjusted_y[i] = room.y[i];
    }

    // If the final sample is a return to the first corner, distribute the XY
    // closure error along the traverse in proportion to cumulative path length.
    // The corrected final sample coincides with point 0 and is omitted from the
    // closed DXF polyline.
    if (has_closure_sample) {
        float total_xy_m = 0.0f;
        for (uint16_t i = 1; i < room.count; ++i) {
            total_xy_m += hypotf(room.x[i] - room.x[i - 1], room.y[i] - room.y[i - 1]);
        }
        if (total_xy_m > 1.0e-4f) {
            const float closure_x_m = room.x[room.count - 1] - room.x[0];
            const float closure_y_m = room.y[room.count - 1] - room.y[0];
            float cumulative_xy_m = 0.0f;
            for (uint16_t i = 1; i < vertex_count; ++i) {
                cumulative_xy_m += hypotf(room.x[i] - room.x[i - 1], room.y[i] - room.y[i - 1]);
                const float fraction = cumulative_xy_m / total_xy_m;
                adjusted_x[i] = room.x[i] - closure_x_m * fraction;
                adjusted_y[i] = room.y[i] - closure_y_m * fraction;
            }
        }
    }

    snprintf(out_file, out_file_size, "/room_%06lu.dxf", static_cast<unsigned long>(room.session_id));
    const std::string path = std::string(SD_MOUNT_POINT) + out_file;
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return false;
    FILE *f = fopen(path.c_str(), "wb");
    bool ok = f != nullptr;
    if (ok) {
        // ASCII DXF, units millimetres ($INSUNITS=4). Keep the measured outline
        // and the closure-adjusted outline on separate layers for inspection.
        ok = fprintf(f,
                     "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1015\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
                     "0\nSECTION\n2\nENTITIES\n"
                     "0\nLWPOLYLINE\n100\nAcDbEntity\n8\nROOM_RAW\n62\n1\n100\nAcDbPolyline\n90\n%u\n70\n1\n",
                     vertex_count) > 0;
        for (uint16_t i = 0; ok && i < vertex_count; ++i) {
            ok = fprintf(f, "10\n%.3f\n20\n%.3f\n",
                         static_cast<double>(room.x[i] * 1000.0f),
                         static_cast<double>(room.y[i] * 1000.0f)) > 0;
        }
        if (ok) {
            ok = fprintf(f, "0\nLWPOLYLINE\n100\nAcDbEntity\n8\nROOM_ADJUSTED\n62\n3\n100\nAcDbPolyline\n90\n%u\n70\n1\n",
                         vertex_count) > 0;
        }
        for (uint16_t i = 0; ok && i < vertex_count; ++i) {
            ok = fprintf(f, "10\n%.3f\n20\n%.3f\n",
                         static_cast<double>(adjusted_x[i] * 1000.0f),
                         static_cast<double>(adjusted_y[i] * 1000.0f)) > 0;
        }
        // Export each corrected edge as an independently selectable wall.
        // Wall labels contain the 1-based wall number and its corrected XY
        // length, so later door/window records can refer to a stable wall ID.
        constexpr float kRadToDeg = 57.2957795131f;
        for (uint16_t i = 0; ok && i < vertex_count; ++i) {
            const uint16_t j = static_cast<uint16_t>((i + 1) % vertex_count);
            const float x1_mm = adjusted_x[i] * 1000.0f;
            const float y1_mm = adjusted_y[i] * 1000.0f;
            const float x2_mm = adjusted_x[j] * 1000.0f;
            const float y2_mm = adjusted_y[j] * 1000.0f;
            const float dx_mm = x2_mm - x1_mm;
            const float dy_mm = y2_mm - y1_mm;
            const float length_mm = hypotf(dx_mm, dy_mm);
            float label_angle_deg = atan2f(dy_mm, dx_mm) * kRadToDeg;
            if (label_angle_deg > 90.0f) label_angle_deg -= 180.0f;
            if (label_angle_deg < -90.0f) label_angle_deg += 180.0f;
            const float mid_x_mm = (x1_mm + x2_mm) * 0.5f;
            const float mid_y_mm = (y1_mm + y2_mm) * 0.5f;
            ok = fprintf(f,
                         "0\nLINE\n100\nAcDbEntity\n8\nROOM_WALLS\n62\n4\n"
                         "10\n%.3f\n20\n%.3f\n11\n%.3f\n21\n%.3f\n"
                         "0\nTEXT\n100\nAcDbEntity\n8\nROOM_WALL_LABELS\n62\n2\n"
                         "10\n%.3f\n20\n%.3f\n40\n30.000\n1\nW%u %.0fMM\n"
                         "50\n%.3f\n72\n1\n73\n2\n11\n%.3f\n21\n%.3f\n",
                         static_cast<double>(x1_mm), static_cast<double>(y1_mm),
                         static_cast<double>(x2_mm), static_cast<double>(y2_mm),
                         static_cast<double>(mid_x_mm), static_cast<double>(mid_y_mm),
                         static_cast<unsigned>(i + 1), static_cast<double>(length_mm),
                         static_cast<double>(label_angle_deg),
                         static_cast<double>(mid_x_mm), static_cast<double>(mid_y_mm)) > 0;
        }
        for (uint16_t i = 0; ok && i < vertex_count; ++i) {
            const float x_mm = adjusted_x[i] * 1000.0f;
            const float y_mm = adjusted_y[i] * 1000.0f;
            ok = fprintf(f,
                         "0\nTEXT\n100\nAcDbEntity\n8\nROOM_CORNER_LABELS\n62\n6\n"
                         "10\n%.3f\n20\n%.3f\n40\n24.000\n1\nP%u\n",
                         static_cast<double>(x_mm + 12.0f), static_cast<double>(y_mm + 12.0f),
                         static_cast<unsigned>(i + 1)) > 0;
        }
        if (ok) ok = fputs("0\nENDSEC\n0\nEOF\n", f) >= 0;
        if (ok) ok = fflush(f) == 0;
        if (ok) ok = fsync(fileno(f)) == 0;
        if (fclose(f) != 0) ok = false;
    }
    xSemaphoreGive(s_sd_log_mutex);
    return ok;
}

static bool room_export_last_from_csv(char *out_file, size_t out_file_size)
{
    if (!s_sd_mounted || !s_sd_log_mutex) return false;
    RoomSurveyState recovered;
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return false;
    OpenLog *log = nullptr;
    for (auto &candidate : s_open_logs) {
        if (strcmp(candidate.name, "room_points.csv") == 0) {
            log = &candidate;
            break;
        }
    }
    if (log && log->file) {
        fflush(log->file);
        fclose(log->file);
        log->file = nullptr;
        log->pending = 0;
    }
    const std::string path = std::string(SD_MOUNT_POINT) + "/room_points.csv";
    FILE *f = fopen(path.c_str(), "rb");
    bool parsed_any = false;
    if (f) {
        char row[384];
        while (fgets(row, sizeof(row), f)) {
            long long t_us = 0;
            unsigned long session_id = 0;
            unsigned point_index = 0;
            long laser_mm = 0;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            float segment = 0.0f, perimeter = 0.0f, closure = 0.0f, area = 0.0f;
            const int fields = sscanf(row, "%lld,%lu,%u,%ld,%f,%f,%f,%f,%f,%f,%f",
                                      &t_us, &session_id, &point_index, &laser_mm,
                                      &x, &y, &z, &segment, &perimeter, &closure, &area);
            if (fields != 11 || session_id == 0 || point_index == 0 || point_index > ROOM_MAX_POINTS) continue;
            if (!parsed_any || recovered.session_id != static_cast<uint32_t>(session_id)) {
                recovered = {};
                recovered.session_id = static_cast<uint32_t>(session_id);
            }
            const uint16_t index = static_cast<uint16_t>(point_index - 1);
            recovered.x[index] = x;
            recovered.y[index] = y;
            recovered.z[index] = z;
            recovered.count = std::max<uint16_t>(recovered.count, static_cast<uint16_t>(index + 1));
            recovered.last_segment_m = segment;
            recovered.open_perimeter_m = perimeter;
            recovered.closure_m = closure;
            recovered.area_xy_m2 = area;
            parsed_any = true;
        }
        fclose(f);
    }
    xSemaphoreGive(s_sd_log_mutex);
    return parsed_any && recovered.count >= 3 && room_write_dxf(recovered, out_file, out_file_size);
}

static bool room_survey_finish()
{
    uint32_t session_id = 0;
    uint16_t valid_count = 0, invalid_count = 0;
    float rotation_deg = 0.0f;
    bool coverage_complete = false;
    char scan_file[48] = "";
    dashboard_lock();
    if (!s_room.active || s_room.scan_valid_count < 30 || !s_room.scan_coverage_complete) {
        dashboard_unlock();
        dashboard_set_error("scan requires >=30 valid points and 355 deg coverage");
        return false;
    }
    session_id = s_room.session_id;
    valid_count = s_room.scan_valid_count;
    invalid_count = s_room.scan_invalid_count;
    rotation_deg = std::fabs(s_room.scan_unwrapped_deg - s_room.scan_start_azimuth_deg);
    coverage_complete = s_room.scan_coverage_complete;
    snprintf(scan_file, sizeof(scan_file), "%s", s_room.scan_file);
    s_room.active = false;
    s_room.complete = true;
    room_publish_locked();
    dashboard_unlock();
    room_scan_close_file();
    char line[256];
    snprintf(line, sizeof(line), "%lld,%lu,%u,%.3f,%u,%u\n",
             static_cast<long long>(esp_timer_get_time()), static_cast<unsigned long>(session_id),
             static_cast<unsigned>(valid_count + invalid_count),
             static_cast<double>(rotation_deg), static_cast<unsigned>(valid_count),
             static_cast<unsigned>(invalid_count));
    dashboard_log_csv("scan_sessions.csv", line);
    dashboard_log_event("room", "scan complete id=" + std::to_string(session_id) +
                                 " valid=" + std::to_string(valid_count) +
                                 " invalid=" + std::to_string(invalid_count) +
                                 " rotation_deg=" + std::to_string(rotation_deg) +
                                 " coverage=" + (coverage_complete ? "complete" : "partial") +
                                 " file=" + scan_file);
    // SD remains the source of truth.  Queue the fully flushed scan for the
    // PC link; failed transfers remain pending and never remove the SD copy.
    pc_link_queue_upload(scan_file);
    s_laser_active_requested.store(false, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    return true;
}

static void room_survey_cancel()
{
    uint32_t session_id = 0;
    dashboard_lock();
    session_id = s_room.session_id;
    s_room.active = false;
    s_room.complete = false;
    room_publish_locked();
    s_dash.room_dxf_saved = false;
    s_dash.room_dxf_file[0] = '\0';
    dashboard_unlock();
    room_scan_close_file();
    // Revoke every request that can keep the continuous laser task alive.
    // The action task owns the UART and will issue iHALT + iLD:0 after the
    // notification below; keeping the UART operation out of the LVGL thread
    // prevents the Cancel button from blocking the UI.
    s_laser_measure_request.store(false, std::memory_order_release);
    s_laser_single_request.store(false, std::memory_order_release);
    s_single_distance_measure_request.store(false, std::memory_order_release);
    s_p2p_measure_request.store(false, std::memory_order_release);
    s_p2p_measure_target.store(-1, std::memory_order_release);
    s_laser_active_requested.store(false, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    dashboard_log_event("room", "cancel id=" + std::to_string(session_id));
}

static bool dashboard_laser_measure_once()
{
    ESP_LOGI("measure", "begin stack_free=%u internal_free=%u",
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
    const int64_t now_us = esp_timer_get_time();
    FusionPose measurement_pose = s_fusion.capturePose();
    if (!measurement_pose.valid || measurement_pose.t_us <= 0 ||
        now_us - measurement_pose.t_us > 500000) {
        dashboard_set_error("P2P magnetic IMU attitude unavailable or stale");
        return false;
    }
    dashboard_lock();
    const bool fixed_room_station = s_room.active && !s_room_moving_mode;
    dashboard_unlock();
    if (fixed_room_station) {
        // Fixed-station mode uses attitude changes and a fixed IMU origin
        // translation drift, so all rays originate from one surveyed station.
        measurement_pose.x = 0.0f;
        measurement_pose.y = 0.0f;
        measurement_pose.z = 0.0f;
    }
    const FusionState before_measurement = s_fusion.snapshot();
    const bool starts_new_session = before_measurement.point_count == 0 || before_measurement.point_count >= 2;
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        dashboard_set_error("laser busy");
        return false;
    }
    const int32_t distance_mm = s_laser_latest_mm;
    const int64_t sample_us = s_laser_latest_us;
    const std::string raw = s_laser_latest_raw;
    xSemaphoreGive(s_laser_mutex);
    if (distance_mm < 0 || sample_us <= 0 || now_us - sample_us > 500000) {
        dashboard_set_error("laser continuous data stale");
        return false;
    }

    dashboard_lock();
    s_dash.t_us = now_us;
    s_dash.laser_ready = true;
    s_dash.laser_busy = false;
    s_dash.laser_mm = distance_mm;
    ++s_dash.laser_count;
    s_dash.last_error[0] = '\0';
    snprintf(s_dash.laser_raw, sizeof(s_dash.laser_raw), "%s", raw.c_str());
    DashboardState snap = s_dash;
    dashboard_unlock();

    if (s_fusion.commitLaserPoint(measurement_pose, distance_mm)) {
        FusionState fused = s_fusion.snapshot();
        uint32_t session_id = 0;
        if (starts_new_session) {
            session_id = measurement_allocate_session_id();
            if (session_id == 0) {
                dashboard_set_error("measurement session ID reservation exhausted");
                return false;
            }
        } else {
            dashboard_lock();
            session_id = s_dash.measure_session_id;
            dashboard_unlock();
        }
        dashboard_lock();
        if (starts_new_session) s_dash.measure_session_id = session_id;
        s_dash.measure_point_count = fused.point_count;
        s_dash.point1_x = fused.point1_x; s_dash.point1_y = fused.point1_y; s_dash.point1_z = fused.point1_z;
        s_dash.point2_x = fused.point2_x; s_dash.point2_y = fused.point2_y; s_dash.point2_z = fused.point2_z;
        s_dash.point_distance_m = fused.point_distance_m;
        snap = s_dash;
        dashboard_unlock();
        char point_line[320];
        snprintf(point_line, sizeof(point_line), "%lld,%u,%ld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                 static_cast<long long>(snap.t_us), snap.measure_point_count,
                 static_cast<long>(snap.laser_mm),
                 static_cast<double>(snap.point1_x), static_cast<double>(snap.point1_y), static_cast<double>(snap.point1_z),
                 static_cast<double>(snap.point2_x), static_cast<double>(snap.point2_y), static_cast<double>(snap.point2_z),
                 static_cast<double>(snap.point_distance_m));
        dashboard_log_csv("measure_points.csv", point_line);

        char session_line[352];
        snprintf(session_line, sizeof(session_line),
                 "%lld,%lu,%u,%ld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                 static_cast<long long>(snap.t_us), static_cast<unsigned long>(snap.measure_session_id),
                 snap.measure_point_count, static_cast<long>(snap.laser_mm),
                 static_cast<double>(snap.point1_x), static_cast<double>(snap.point1_y), static_cast<double>(snap.point1_z),
                 static_cast<double>(snap.point2_x), static_cast<double>(snap.point2_y), static_cast<double>(snap.point2_z),
                 static_cast<double>(snap.point_distance_m));
        dashboard_log_csv("measure_sessions.csv", session_line);
        if (starts_new_session) {
            dashboard_log_event("measure_session", "start id=" + std::to_string(snap.measure_session_id));
        } else if (snap.measure_point_count == 2) {
            dashboard_log_event("measure_session",
                                "complete id=" + std::to_string(snap.measure_session_id) +
                                " distance_mm=" + std::to_string(snap.point_distance_m * 1000.0f));
        }
        if (snap.room_active) {
            const bool first = fused.point_count == 1;
            room_survey_append_point(snap.t_us, snap.laser_mm,
                                     first ? fused.point1_x : fused.point2_x,
                                     first ? fused.point1_y : fused.point2_y,
                                     first ? fused.point1_z : fused.point2_z);
        }
    }

    char line[360];
    snprintf(line, sizeof(line), "%lld,%ld,%s,%s\n",
             static_cast<long long>(snap.t_us),
             static_cast<long>(snap.laser_mm),
             "continuous_ascii",
             snap.laser_raw);
    dashboard_log_csv("laser.csv", line);
    dashboard_log_event("laser", std::to_string(snap.laser_mm) + "mm continuous_ascii");
    ESP_LOGI("measure", "complete id=%lu point=%u stack_free=%u internal_free=%u",
             static_cast<unsigned long>(snap.measure_session_id), snap.measure_point_count,
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
    return true;
}

static bool dashboard_laser_measure_single()
{
    const int64_t now_us = esp_timer_get_time();
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        dashboard_set_error("laser busy");
        return false;
    }
    const int32_t distance_mm = s_laser_latest_mm;
    const int64_t sample_us = s_laser_latest_us;
    const std::string raw = s_laser_latest_raw;
    xSemaphoreGive(s_laser_mutex);
    if (distance_mm < 0 || sample_us <= 0 || now_us - sample_us > 500000) {
        dashboard_set_error("laser continuous data stale");
        return false;
    }
    dashboard_lock();
    s_dash.t_us = now_us;
    s_dash.laser_ready = true;
    s_dash.laser_busy = false;
    s_dash.laser_mm = distance_mm;
    ++s_dash.laser_count;
    s_dash.last_error[0] = '\0';
    snprintf(s_dash.laser_raw, sizeof(s_dash.laser_raw), "%s", raw.c_str());
    dashboard_unlock();
    char line[360];
    snprintf(line, sizeof(line), "%lld,%ld,%s,%s\n",
             static_cast<long long>(now_us), static_cast<long>(distance_mm),
             "single_ascii", raw.c_str());
    dashboard_log_csv("laser.csv", line);
    dashboard_log_event("single", std::to_string(distance_mm) + "mm laser_confirmed");
    // AI assist uses this legacy quick capture. Formal single-distance records
    // are only written after the user explicitly confirms SAVE on the product
    // state-machine page.
    return true;
}

static bool dashboard_laser_start_continuous(bool fast_20hz)
{
    if (!s_laser_active_requested.load(std::memory_order_acquire)) return false;
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return false;
    }
    bool activated = false;
    esp_err_t err = uart_reconfigure(LASER_UART_NUM, s_laser_active_tx, s_laser_active_rx,
                                     s_laser_active_baud, &s_uart_laser_ready);
    if (err == ESP_OK) {
        uart_flush_input(LASER_UART_NUM);
        const char *halt = "iHALT\r\n";
        const char *aim = "iLD:1\r\n";
        const char *continuous = fast_20hz ? "iFACM\r\n" : "iACM\r\n";
        uart_write_bytes(LASER_UART_NUM, halt, strlen(halt));
        vTaskDelay(pdMS_TO_TICKS(100));
        uart_flush_input(LASER_UART_NUM);
        if (fast_20hz && !s_laser_fast_rate_configured) {
            // L1 manual section 6.1.7: iSET:7,20 selects the approximately
            // 20 Hz output rate, and that setting applies to iFACM only.
            // Configure it when a room scan starts instead of claiming that
            // the precision-oriented iACM command is a 20 Hz stream.
            const char *rate = "iSET:7,20\r\n";
            uart_write_bytes(LASER_UART_NUM, rate, strlen(rate));
            uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(100));
            vTaskDelay(pdMS_TO_TICKS(80));
            uart_flush_input(LASER_UART_NUM);
            s_laser_fast_rate_configured = true;
        }
        if (s_laser_active_requested.load(std::memory_order_acquire)) {
            uart_write_bytes(LASER_UART_NUM, aim, strlen(aim));
            vTaskDelay(pdMS_TO_TICKS(80));
        }
        if (s_laser_active_requested.load(std::memory_order_acquire)) {
            uart_write_bytes(LASER_UART_NUM, continuous, strlen(continuous));
            activated = true;
        } else {
            const char *off = "iLD:0\r\n";
            uart_write_bytes(LASER_UART_NUM, halt, strlen(halt));
            uart_write_bytes(LASER_UART_NUM, off, strlen(off));
        }
        uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(200));
        s_laser_stream_line.clear();
        s_laser_stream_bytes.clear();
        s_laser_latest_mm = -1;
        s_laser_latest_us = 0;
        s_laser_latest_raw.clear();
    }
    xSemaphoreGive(s_laser_mutex);
    s_laser_continuous_active.store(activated, std::memory_order_release);
    return err == ESP_OK && activated;
}

static bool dashboard_laser_stop_continuous(bool reconfigure)
{
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }
    esp_err_t err = ESP_OK;
    if (reconfigure) {
        err = uart_reconfigure(LASER_UART_NUM, s_laser_active_tx, s_laser_active_rx,
                                s_laser_active_baud, &s_uart_laser_ready);
    } else if (!s_uart_laser_ready) {
        err = ESP_ERR_INVALID_STATE;
    }
    if (err == ESP_OK) {
        const char *halt = "iHALT\r\n";
        const char *off = "iLD:0\r\n";
        uart_write_bytes(LASER_UART_NUM, halt, strlen(halt));
        uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(20));
        uart_write_bytes(LASER_UART_NUM, off, strlen(off));
        uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(100));
        // iLD:0 only changes the current state.  iSET:10,0 additionally
        // persists the module's power-on laser setting as OFF, so the L1 does
        // not light during the next MCU boot before application code runs.
        if (!s_laser_startup_off_configured) {
            // Do not mark the setting as persisted until the module replies.
            // A cold-powered L1 can need a short settling time; leaving this
            // flag false on timeout lets the later runtime initialization
            // retry instead of silently assuming the command was accepted.
            vTaskDelay(pdMS_TO_TICKS(20));
            const bool startup_off_ok = laser_exchange_locked("iSET:10,0", 400);
            if (startup_off_ok) {
                s_laser_startup_off_configured = true;
                ESP_LOGI("laser_mode", "module acknowledged power-on laser OFF configuration");
            } else {
                ESP_LOGW("laser_mode", "iSET:10,0 received no response; will retry after module startup");
            }
        }
        uart_flush_input(LASER_UART_NUM);
    }
    s_laser_stream_line.clear();
    s_laser_stream_bytes.clear();
    s_laser_latest_mm = -1;
    s_laser_latest_us = 0;
    s_laser_latest_raw.clear();
    xSemaphoreGive(s_laser_mutex);
    s_laser_continuous_active.store(false, std::memory_order_release);
    dashboard_lock();
    s_dash.laser_ready = false;
    s_dash.laser_busy = false;
    s_dash.laser_mm = -1;
    s_dash.laser_raw[0] = '\0';
    dashboard_unlock();
    return err == ESP_OK;
}

static void dashboard_laser_poll_continuous()
{
    if (!s_laser_mutex || xSemaphoreTake(s_laser_mutex, 0) != pdTRUE) return;
    uint8_t bytes[128];
    int n = uart_read_bytes(LASER_UART_NUM, bytes, sizeof(bytes), 0);
    bool parsed_this_poll = false;
    if (n > 0) {
        s_laser_stream_bytes.insert(s_laser_stream_bytes.end(), bytes, bytes + n);
        if (s_laser_stream_bytes.size() > 512) {
            s_laser_stream_bytes.erase(s_laser_stream_bytes.begin(),
                                       s_laser_stream_bytes.end() - 256);
        }
        // Some L1 firmware revisions stream the documented B4 69 binary
        // frame without CR/LF. Parse the complete byte window as well as the
        // ASCII line path so those revisions no longer appear silent.
        const LaserParseResult binary = laser_parse_measurement(s_laser_stream_bytes);
        if (binary.found && binary.source == "hex_b469") {
            s_laser_latest_mm = binary.distance_mm;
            s_laser_latest_us = esp_timer_get_time();
            s_laser_latest_raw = "B469 binary";
            parsed_this_poll = true;
            dashboard_lock();
            s_dash.laser_ready = true;
            s_dash.laser_mm = binary.distance_mm;
            snprintf(s_dash.laser_raw, sizeof(s_dash.laser_raw), "B469 binary");
            dashboard_unlock();
            s_laser_stream_bytes.clear();
        }
    }
    for (int i = 0; i < n; ++i) {
        const uint8_t c = bytes[i];
        if (c == '\r' || c == '\n') {
            if (!s_laser_stream_line.empty()) {
                LaserParseResult parsed;
                if (laser_parse_ascii_line(s_laser_stream_line, parsed) && parsed.found) {
                    parsed_this_poll = true;
                    s_laser_latest_mm = parsed.distance_mm;
                    s_laser_latest_us = esp_timer_get_time();
                    s_laser_latest_raw = s_laser_stream_line;
                    dashboard_lock();
                    s_dash.laser_ready = true;
                    s_dash.laser_mm = parsed.distance_mm;
                    snprintf(s_dash.laser_raw, sizeof(s_dash.laser_raw), "%s", s_laser_stream_line.c_str());
                    dashboard_unlock();
                }
                s_laser_stream_line.clear();
            }
        } else if (c >= 32 && c <= 126) {
            if (s_laser_stream_line.size() >= 159) s_laser_stream_line.erase(0, 80);
            s_laser_stream_line.push_back(static_cast<char>(c));
        }
    }
    if (n > 0 && !parsed_this_poll) {
        static int64_t last_unparsed_log_us = 0;
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_unparsed_log_us >= 2000000) {
            ESP_LOGW("laser_uart", "received %d bytes but no distance parsed", n);
            ESP_LOG_BUFFER_HEXDUMP("laser_uart_raw", bytes, n, ESP_LOG_WARN);
            last_unparsed_log_us = now_us;
        }
    }
    xSemaphoreGive(s_laser_mutex);
}

static int32_t single_reference_offset_mm(DistanceReference reference)
{
    float offset_mm;
    switch (reference) {
    case DistanceReference::FRONT: offset_mm = SINGLE_FRONT_REFERENCE_OFFSET_MM; break;
    case DistanceReference::TRIPOD: offset_mm = SINGLE_TRIPOD_REFERENCE_OFFSET_MM; break;
    default: offset_mm = SINGLE_REAR_REFERENCE_OFFSET_MM; break;
    }
    return static_cast<int32_t>(std::llround(static_cast<double>(offset_mm)));
}

static void single_distance_apply_action(SingleDistanceAction action)
{
    switch (action) {
    case SingleDistanceAction::LASER_ON:
        s_single_distance_cancel_request.store(false, std::memory_order_release);
        s_laser_active_requested.store(true, std::memory_order_release);
        break;
    case SingleDistanceAction::LASER_OFF:
        s_laser_active_requested.store(false, std::memory_order_release);
        break;
    case SingleDistanceAction::START_MEASUREMENT:
        s_single_distance_cancel_request.store(false, std::memory_order_release);
        s_single_distance_measure_request.store(true, std::memory_order_release);
        break;
    case SingleDistanceAction::CANCEL_MEASUREMENT:
        s_single_distance_cancel_request.store(true, std::memory_order_release);
        s_laser_active_requested.store(false, std::memory_order_release);
        break;
    default:
        return;
    }
    if (s_action_task) xTaskNotifyGive(s_action_task);
}

static bool dashboard_single_distance_measure()
{
    SingleDistanceSnapshot session;
    dashboard_lock();
    session = s_single_distance.snapshot();
    dashboard_unlock();
    if (session.state != SingleDistanceState::MEASURING || session.measure_request_us <= 0) return false;

    constexpr int64_t kMeasureTimeoutUs = 5000000;
    const int64_t request_us = session.measure_request_us;
    int32_t distance_mm = -1;
    int64_t sample_us = 0;
    std::string raw;
    int64_t last_single_command_us = 0;
    while (esp_timer_get_time() - request_us < kMeasureTimeoutUs) {
        if (s_single_distance_cancel_request.load(std::memory_order_acquire) ||
            !s_laser_active_requested.load(std::memory_order_acquire)) {
            dashboard_log_event("single", "measurement cancelled");
            return false;
        }
        dashboard_laser_poll_continuous();
        const int64_t loop_us = esp_timer_get_time();
        if (loop_us - request_us >= 700000 &&
            (last_single_command_us == 0 || loop_us - last_single_command_us >= 1000000) &&
            s_laser_mutex && xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Continuous output differs between L1 firmware revisions.  iSM
            // is a safe one-shot fallback and its response is consumed by the
            // same ASCII/binary parser on the following iterations.
            const char *single = "iSM\r\n";
            uart_write_bytes(LASER_UART_NUM, single, strlen(single));
            uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(100));
            xSemaphoreGive(s_laser_mutex);
            last_single_command_us = loop_us;
            ESP_LOGI("laser_uart", "continuous sample stale; sent iSM fallback");
        }
        if (s_laser_mutex && xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (s_laser_latest_us >= request_us && s_laser_latest_mm > 0) {
                distance_mm = s_laser_latest_mm;
                sample_us = s_laser_latest_us;
                raw = s_laser_latest_raw;
            }
            xSemaphoreGive(s_laser_mutex);
        }
        if (distance_mm > 0) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (distance_mm <= 0) {
        dashboard_lock();
        const bool failed = s_single_distance.measurement_failed(
            SingleDistanceError::MEASURE_TIMEOUT, static_cast<int32_t>(ESP_ERR_TIMEOUT));
        s_dash.laser_busy = false;
        dashboard_unlock();
        s_laser_active_requested.store(false, std::memory_order_release);
        if (failed) dashboard_set_error("single measurement timeout: no fresh return within 5 seconds");
        return false;
    }

    const int64_t completed_us = esp_timer_get_time();
    SingleDistanceResult result;
    struct timeval wall_time = {};
    gettimeofday(&wall_time, nullptr);
    result.timestamp_ms = wall_time.tv_sec >= 1609459200
        ? static_cast<int64_t>(wall_time.tv_sec) * 1000 + wall_time.tv_usec / 1000
        : completed_us / 1000;
    result.reference = session.measurement_reference;
    result.distance_mm = distance_mm + single_reference_offset_mm(result.reference);
    result.measure_time_ms = static_cast<uint16_t>(std::min<int64_t>(
        UINT16_MAX, std::max<int64_t>(0, (sample_us - request_us) / 1000)));
    result.raw_error_code = 0;
    // Bind the scene seen at the time of the accepted laser reading to this
    // temporary result. A photo failure must not discard a valid range.
    std::string image_path;
    result.camera_frame_saved = dashboard_capture_photo(&image_path, "single");
    if (result.camera_frame_saved) {
        snprintf(result.image_path, sizeof(result.image_path), "%s", image_path.c_str());
        result.quality = static_cast<uint8_t>(CAMERA_HTTP_JPEG_QUALITY);
    }
    dashboard_lock();
    result.pitch_deg = s_dash.fusion_pitch * 57.2957795f;
    result.roll_deg = s_dash.fusion_roll * 57.2957795f;
    result.yaw_deg = s_dash.fusion_yaw * 57.2957795f;
    const bool accepted = s_single_distance.measurement_succeeded(result);
    if (accepted) {
        s_dash.t_us = completed_us;
        s_dash.laser_ready = true;
        s_dash.laser_busy = false;
        s_dash.laser_mm = result.distance_mm;
        ++s_dash.laser_count;
        s_dash.last_error[0] = '\0';
        snprintf(s_dash.laser_raw, sizeof(s_dash.laser_raw), "%s", raw.c_str());
    }
    dashboard_unlock();
    s_laser_active_requested.store(false, std::memory_order_release);
    if (!accepted) return false;

    char line[360];
    snprintf(line, sizeof(line), "%lld,%ld,%s,%s\n",
             static_cast<long long>(completed_us), static_cast<long>(result.distance_mm),
             "single_state_machine", raw.c_str());
    dashboard_log_csv("laser.csv", line);
    dashboard_log_event("single", "result temp distance_mm=" + std::to_string(result.distance_mm) +
                                      " reference=" + std::to_string(static_cast<unsigned>(result.reference)) +
                                      " measure_ms=" + std::to_string(result.measure_time_ms) +
                                      (result.camera_frame_saved ? " photo=" + image_path : " photo=disabled_or_failed"));
    return true;
}

static bool dashboard_single_distance_save()
{
    SingleDistanceSnapshot session;
    bool sd_ready = false;
    dashboard_lock();
    session = s_single_distance.snapshot();
    sd_ready = s_dash.sd_ready;
    dashboard_unlock();
    if (session.state != SingleDistanceState::SAVING || !session.result_valid) return false;

    if (!sd_ready || !s_measurement_store.ready()) {
        dashboard_lock();
        s_single_distance.save_failed(SingleDistanceError::STORAGE_UNAVAILABLE,
                                      static_cast<int32_t>(ESP_ERR_INVALID_STATE));
        dashboard_unlock();
        dashboard_set_error("single result kept in RAM: SD record store unavailable");
        return false;
    }

    MeasurementRecord draft;
    draft.t_us = session.result.timestamp_ms * 1000;
    draft.distance_mm = session.result.distance_mm;
    draft.reference = session.result.reference;
    draft.pitch_deg = session.result.pitch_deg;
    draft.roll_deg = session.result.roll_deg;
    draft.yaw_deg = session.result.yaw_deg;
    draft.camera_frame_saved = session.result.camera_frame_saved;
    snprintf(draft.image_path, sizeof(draft.image_path), "%s", session.result.image_path);
    draft.quality = session.result.quality;
    draft.measure_time_ms = session.result.measure_time_ms;
    draft.raw_error_code = session.result.raw_error_code;

    MeasurementRecord saved;
    const esp_err_t err = s_measurement_store.append(draft, &saved);
    if (err != ESP_OK) {
        dashboard_lock();
        s_single_distance.save_failed(SingleDistanceError::SAVE_FAILED, static_cast<int32_t>(err));
        dashboard_unlock();
        dashboard_set_error("single result kept in RAM: record save failed: " +
                            std::string(esp_err_to_name(err)));
        return false;
    }

    dashboard_lock();
    s_single_distance.save_succeeded(saved.id);
    s_dash.last_saved_record_id = saved.id;
    s_dash.last_error[0] = '\0';
    dashboard_unlock();
    publish_measurement_records();
    dashboard_log_event("record", "saved id=" + std::to_string(saved.id) +
                                      " distance_mm=" + std::to_string(saved.distance_mm) +
                                      " reference=" + std::to_string(static_cast<unsigned>(saved.reference)));
    return true;
}

static bool dashboard_p2p_measure(int target);
static bool dashboard_p2p_save();

static void p2p_recalculate_locked()
{
    if (!s_dash.p2p_a_valid || !s_dash.p2p_b_valid) {
        s_dash.p2p_space_distance_m = -1.0f;
        s_dash.p2p_horizontal_distance_m = -1.0f;
        s_dash.p2p_height_diff_m = 0.0f;
        return;
    }
    if (!s_fusion.solveRelativePair(s_dash.p2p_pose_a, s_dash.p2p_distance_a_mm,
                                    s_dash.p2p_pose_b, s_dash.p2p_distance_b_mm,
                                    &s_dash.p2p_space_distance_m,
                                    &s_dash.p2p_horizontal_distance_m,
                                    &s_dash.p2p_height_diff_m,
                                    &s_dash.p2p_relative_angle_deg)) {
        s_dash.p2p_space_distance_m = -1.0f;
        return;
    }
    const float avg_accel = s_dash.p2p_accel_sample_count
        ? s_dash.p2p_accel_deviation_sum / s_dash.p2p_accel_sample_count : 0.0f;
    // Soft classification only: even HIGH risk remains measurable.  Linear
    // acceleration cannot observe slow/constant-velocity translation, so this
    // is explicitly a warning score rather than a position correction.
    s_dash.p2p_motion_risk = (s_dash.p2p_max_linear_accel_mps2 > 2.0f || avg_accel > 0.8f) ? 2 :
                             (s_dash.p2p_max_linear_accel_mps2 > 0.8f || avg_accel > 0.3f) ? 1 : 0;
}

static bool p2p_pose_at(int64_t target_us, FusionPose *pose, uint16_t *error_ms)
{
    if (!pose || target_us <= 0) return false;
    std::array<TimedQuaternion, P2P_IMU_HISTORY_CAPACITY> history = {};
    size_t head = 0, count = 0;
    taskENTER_CRITICAL(&s_game_rv_mux);
    history = s_game_rv_history;
    head = s_game_rv_head;
    count = s_game_rv_count;
    taskEXIT_CRITICAL(&s_game_rv_mux);
    if (!count) return false;
    TimedQuaternion before = {}, after = {};
    bool have_before = false, have_after = false;
    for (size_t i = 0; i < count; ++i) {
        const size_t index = (head + P2P_IMU_HISTORY_CAPACITY - count + i) % P2P_IMU_HISTORY_CAPACITY;
        const TimedQuaternion &sample = history[index];
        if (sample.t_us <= target_us && (!have_before || sample.t_us > before.t_us)) {
            before = sample; have_before = true;
        }
        if (sample.t_us >= target_us && (!have_after || sample.t_us < after.t_us)) {
            after = sample; have_after = true;
        }
    }
    TimedQuaternion matched;
    int64_t sync_error_us = 0;
    if (have_before && have_after && after.t_us > before.t_us) {
        float bx = before.qi, by = before.qj, bz = before.qk, bw = before.qr;
        if (bx * after.qi + by * after.qj + bz * after.qk + bw * after.qr < 0.0f) {
            bx = -bx; by = -by; bz = -bz; bw = -bw;
        }
        const float alpha = std::clamp(static_cast<float>(target_us - before.t_us) /
                                       static_cast<float>(after.t_us - before.t_us), 0.0f, 1.0f);
        matched.t_us = target_us;
        matched.qi = bx + (after.qi - bx) * alpha;
        matched.qj = by + (after.qj - by) * alpha;
        matched.qk = bz + (after.qk - bz) * alpha;
        matched.qr = bw + (after.qr - bw) * alpha;
        matched.accuracy = std::min(before.accuracy, after.accuracy);
        sync_error_us = std::max(target_us - before.t_us, after.t_us - target_us);
    } else {
        matched = have_before ? before : after;
        sync_error_us = std::llabs(target_us - matched.t_us);
    }
    if (sync_error_us > 150000) return false;
    *pose = s_fusion.bodyPoseFromSensorQuaternion(target_us, matched.qi, matched.qj,
                                                  matched.qk, matched.qr);
    if (error_ms) *error_ms = static_cast<uint16_t>(std::min<int64_t>(sync_error_us / 1000, UINT16_MAX));
    return pose->valid;
}

static bool dashboard_p2p_measure(int target)
{
    if (target < 0 || target > 1) return false;
    const int64_t request_us = esp_timer_get_time();
    int32_t distance_mm = -1;
    int64_t sample_us = 0;
    int64_t last_single_command_us = 0;
    while (esp_timer_get_time() - request_us < 5000000) {
        if (!s_laser_active_requested.load(std::memory_order_acquire)) return false;
        dashboard_laser_poll_continuous();
        const int64_t loop_us = esp_timer_get_time();
        if (loop_us - request_us >= 700000 &&
            (last_single_command_us == 0 || loop_us - last_single_command_us >= 1000000) &&
            s_laser_mutex && xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            static constexpr char command[] = "iSM\r\n";
            uart_write_bytes(LASER_UART_NUM, command, sizeof(command) - 1);
            uart_wait_tx_done(LASER_UART_NUM, pdMS_TO_TICKS(100));
            xSemaphoreGive(s_laser_mutex);
            last_single_command_us = loop_us;
        }
        if (s_laser_mutex && xSemaphoreTake(s_laser_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (s_laser_latest_us >= request_us && s_laser_latest_mm > 0) {
                distance_mm = s_laser_latest_mm;
                sample_us = s_laser_latest_us;
            }
            xSemaphoreGive(s_laser_mutex);
        }
        if (distance_mm > 0) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Give the 50 Hz Game RV stream one report interval to bracket the UART
    // receive timestamp.  This keeps range/attitude pairing independent of
    // SD photo latency and normally enables interpolation instead of nearest.
    vTaskDelay(pdMS_TO_TICKS(25));
    FusionPose pose;
    uint16_t sync_error_ms = 0;
    dashboard_lock();
    const bool tripod_mode = s_dash.p2p_tripod_mode;
    dashboard_unlock();
    bool uses_game_rv = false;
    if (tripod_mode) {
        bool require_game_rv = true;
        if (target == 1) {
            dashboard_lock();
            require_game_rv = s_dash.p2p_pose_a_uses_game_rv;
            dashboard_unlock();
        }
        if (require_game_rv) uses_game_rv = p2p_pose_at(sample_us, &pose, &sync_error_ms);
        if (!uses_game_rv) {
            if (target == 1 && require_game_rv) {
                dashboard_lock();
                s_dash.p2p_stage = P2pStage::ERROR;
                dashboard_unlock();
                s_laser_active_requested.store(false, std::memory_order_release);
                dashboard_set_error("P2P Game RV timestamp match unavailable");
                return false;
            }
            pose = s_fusion.capturePose();
            sync_error_ms = static_cast<uint16_t>(std::min<int64_t>(
                std::llabs(sample_us - pose.t_us) / 1000, UINT16_MAX));
        }
    } else if (!tripod_mode) {
        pose = s_fusion.capturePose();
        sync_error_ms = static_cast<uint16_t>(std::min<int64_t>(
            std::llabs(sample_us - pose.t_us) / 1000, UINT16_MAX));
    }
    const int64_t now_us = esp_timer_get_time();
    if (distance_mm <= 0 || !pose.valid || pose.t_us <= 0 || now_us - pose.t_us > 500000) {
        dashboard_lock();
        s_dash.p2p_stage = P2pStage::ERROR;
        dashboard_unlock();
        s_laser_active_requested.store(false, std::memory_order_release);
        dashboard_set_error(distance_mm <= 0 ? "P2P ranging timeout" : "P2P IMU attitude unavailable or stale");
        return false;
    }
    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (!s_fusion.projectLaserPoint(pose, distance_mm, &x, &y, &z)) return false;

    // Bind attitude and timestamp to the accepted range sample, before SD I/O.
    const FusionState attitude = s_fusion.snapshot();
    struct timeval wall_time = {};
    gettimeofday(&wall_time, nullptr);
    const int64_t timestamp_ms = wall_time.tv_sec >= 1609459200
        ? static_cast<int64_t>(wall_time.tv_sec) * 1000 + wall_time.tv_usec / 1000
        : sample_us / 1000;
    std::string image_path;
    const bool photo_saved = dashboard_capture_photo(&image_path, target == 0 ? "p2p_a" : "p2p_b");
    dashboard_lock();
    if (target == 0) {
        s_dash.p2p_a_valid = true;
        s_dash.p2p_distance_a_mm = distance_mm;
        s_dash.p2p_timestamp_a_ms = timestamp_ms;
        s_dash.p2p_pitch_a_deg = attitude.pitch * 57.2957795f;
        s_dash.p2p_roll_a_deg = attitude.roll * 57.2957795f;
        s_dash.p2p_yaw_a_deg = attitude.yaw * 57.2957795f;
        s_dash.p2p_ax_m = x; s_dash.p2p_ay_m = y; s_dash.p2p_az_m = z;
        s_dash.p2p_pose_a = pose;
        s_dash.p2p_pose_a_uses_game_rv = uses_game_rv;
        s_dash.p2p_motion_start_us = sample_us;
        s_dash.p2p_max_linear_accel_mps2 = 0.0f;
        s_dash.p2p_accel_deviation_sum = 0.0f;
        s_dash.p2p_accel_sample_count = 0;
        s_dash.p2p_max_gyro_dps = 0.0f;
        snprintf(s_dash.p2p_image_a, sizeof(s_dash.p2p_image_a), "%s", photo_saved ? image_path.c_str() : "");
        s_dash.p2p_stage = s_dash.p2p_b_valid ? P2pStage::COMPLETE : P2pStage::WAIT_B;
    } else {
        s_dash.p2p_b_valid = true;
        s_dash.p2p_distance_b_mm = distance_mm;
        s_dash.p2p_timestamp_b_ms = timestamp_ms;
        s_dash.p2p_pitch_b_deg = attitude.pitch * 57.2957795f;
        s_dash.p2p_roll_b_deg = attitude.roll * 57.2957795f;
        s_dash.p2p_yaw_b_deg = attitude.yaw * 57.2957795f;
        s_dash.p2p_bx_m = x; s_dash.p2p_by_m = y; s_dash.p2p_bz_m = z;
        s_dash.p2p_pose_b = pose;
        s_dash.p2p_motion_start_us = 0;
        snprintf(s_dash.p2p_image_b, sizeof(s_dash.p2p_image_b), "%s", photo_saved ? image_path.c_str() : "");
        s_dash.p2p_stage = s_dash.p2p_a_valid ? P2pStage::COMPLETE : P2pStage::WAIT_A;
    }
    p2p_recalculate_locked();
    s_dash.p2p_imu_sync_error_ms = sync_error_ms;
    s_dash.measure_point_count = (s_dash.p2p_a_valid ? 1 : 0) + (s_dash.p2p_b_valid ? 1 : 0);
    s_dash.point_distance_m = s_dash.p2p_space_distance_m;
    s_dash.laser_mm = distance_mm;
    s_dash.laser_busy = false;
    s_dash.last_error[0] = '\0';
    const float logged_angle = s_dash.p2p_relative_angle_deg;
    const float logged_accel = s_dash.p2p_max_linear_accel_mps2;
    const float logged_gyro = s_dash.p2p_max_gyro_dps;
    const uint8_t logged_risk = s_dash.p2p_motion_risk;
    dashboard_unlock();
    s_laser_active_requested.store(false, std::memory_order_release);
    if (target == 1) {
        char quality[224];
        snprintf(quality, sizeof(quality),
                 "source=%s sync_ms=%u angle_deg=%.3f max_linear_accel=%.3f max_gyro_dps=%.3f risk=%u",
                 tripod_mode ? (uses_game_rv ? "game_rv" : "magnetic_fallback") : "magnetic_handheld",
                 static_cast<unsigned>(sync_error_ms), static_cast<double>(logged_angle),
                 static_cast<double>(logged_accel), static_cast<double>(logged_gyro),
                 static_cast<unsigned>(logged_risk));
        dashboard_log_event("p2p_quality", quality);
    }
    dashboard_log_event("p2p", std::string(target == 0 ? "A" : "B") +
                               " distance_mm=" + std::to_string(distance_mm) +
                               (photo_saved ? " photo=" + image_path : " photo_failed"));
    return true;
}

static bool dashboard_p2p_save()
{
    dashboard_lock();
    if (s_dash.p2p_stage == P2pStage::SAVED) {
        dashboard_unlock();
        return true;
    }
    if (!s_dash.p2p_a_valid || !s_dash.p2p_b_valid || s_dash.p2p_space_distance_m <= 0.0f) {
        dashboard_unlock();
        return false;
    }
    s_dash.p2p_stage = P2pStage::SAVING;
    MeasurementRecord draft;
    draft.type = MeasurementRecord::Type::P2P;
    draft.t_us = std::max(s_dash.p2p_timestamp_a_ms, s_dash.p2p_timestamp_b_ms) * 1000;
    draft.distance_mm = static_cast<int32_t>(std::lround(s_dash.p2p_space_distance_m * 1000.0f));
    draft.distance_a_mm = s_dash.p2p_distance_a_mm;
    draft.distance_b_mm = s_dash.p2p_distance_b_mm;
    draft.horizontal_mm = static_cast<int32_t>(std::lround(s_dash.p2p_horizontal_distance_m * 1000.0f));
    draft.height_diff_mm = static_cast<int32_t>(std::lround(s_dash.p2p_height_diff_m * 1000.0f));
    draft.timestamp_a_ms = s_dash.p2p_timestamp_a_ms;
    draft.timestamp_b_ms = s_dash.p2p_timestamp_b_ms;
    draft.pitch_a_deg = s_dash.p2p_pitch_a_deg; draft.roll_a_deg = s_dash.p2p_roll_a_deg; draft.yaw_a_deg = s_dash.p2p_yaw_a_deg;
    draft.pitch_b_deg = s_dash.p2p_pitch_b_deg; draft.roll_b_deg = s_dash.p2p_roll_b_deg; draft.yaw_b_deg = s_dash.p2p_yaw_b_deg;
    snprintf(draft.image_path, sizeof(draft.image_path), "%s", s_dash.p2p_image_a);
    snprintf(draft.image_path_b, sizeof(draft.image_path_b), "%s", s_dash.p2p_image_b);
    draft.camera_frame_saved = draft.image_path[0] != '\0';
    draft.quality = static_cast<uint8_t>(s_dash.p2p_motion_risk == 0 ? 3 :
                                         s_dash.p2p_motion_risk == 1 ? 2 : 1);
    draft.raw_error_code = (s_dash.p2p_tripod_mode ? 0x10000 : 0x20000) |
                           (static_cast<int32_t>(s_dash.p2p_motion_risk) << 8) |
                           std::min<int32_t>(s_dash.p2p_imu_sync_error_ms, 255);
    dashboard_unlock();
    MeasurementRecord saved;
    const esp_err_t err = s_measurement_store.append(draft, &saved);
    dashboard_lock();
    s_dash.p2p_stage = err == ESP_OK ? P2pStage::SAVED : P2pStage::ERROR;
    if (err == ESP_OK) s_dash.last_saved_record_id = saved.id;
    dashboard_unlock();
    if (err != ESP_OK) {
        dashboard_set_error("P2P record save failed: " + std::string(esp_err_to_name(err)));
        return false;
    }
    publish_measurement_records();
    dashboard_log_event("p2p", "saved record id=" + std::to_string(saved.id));
    return true;
}

static void dashboard_key_task(void *)
{
    esp_err_t err = s_pca.begin();
    if (err != ESP_OK) {
        dashboard_set_error("PCA9557 init failed: " + esp_err_str(err));
    }
    bool prev_measure = false;
    bool prev_back = false;
    bool prev_ok = false;
    // 长按检测:记录每个按键当前按住时长(ms);释放时判定——
    // 按住 <1500ms 计短按,≥1500ms 计长按(避免长短按双触发)。
    static constexpr int64_t kLongPressThresholdMs = 1500;
    int64_t hold_measure_ms = -1;
    int64_t hold_back_ms = -1;
    int64_t hold_ok_ms = -1;
    // 抑制标志:按住期间为 true,防止按下瞬间误计短按
    bool armed_measure = false, armed_back = false, armed_ok = false;
    while (true) {
        uint8_t input = 0xFF;
        if (s_pca.readInput(&input) == ESP_OK) {
            bool measure = KEY_ACTIVE_LOW ? !(input & (1u << PCA_IO_KEY_MEASURE)) : (input & (1u << PCA_IO_KEY_MEASURE));
            bool back = KEY_ACTIVE_LOW ? !(input & (1u << PCA_IO_KEY_BACK)) : (input & (1u << PCA_IO_KEY_BACK));
            bool ok = KEY_ACTIVE_LOW ? !(input & (1u << PCA_IO_KEY_OK)) : (input & (1u << PCA_IO_KEY_OK));
            dashboard_lock();
            s_dash.key_measure = measure;
            s_dash.key_back = back;
            s_dash.key_ok = ok;
            s_dash.key_raw = input;
            // 按下沿:开始计时并置位(暂不计短按)
            if (measure && !prev_measure) {
                hold_measure_ms = 0;
                armed_measure = true;
            }
            if (back && !prev_back) {
                hold_back_ms = 0;
                armed_back = true;
            }
            if (ok && !prev_ok) {
                hold_ok_ms = 0;
                armed_ok = true;
            }
            // 按住中:累计时长
            if (measure && hold_measure_ms >= 0) hold_measure_ms += 40;
            if (back && hold_back_ms >= 0) hold_back_ms += 40;
            if (ok && hold_ok_ms >= 0) hold_ok_ms += 40;
            // 释放沿:按住 <1500ms 计短按,≥1500ms 计长按(中间态归入短按,
            // 避免用户普通按压略慢时事件丢失)
            if (!measure && prev_measure && armed_measure) {
                armed_measure = false;
                if (hold_measure_ms >= kLongPressThresholdMs) ++s_dash.long_measure_press_count;
                else ++s_dash.measure_press_count;
                hold_measure_ms = -1;
            }
            if (!back && prev_back && armed_back) {
                armed_back = false;
                if (hold_back_ms >= kLongPressThresholdMs) ++s_dash.long_back_press_count;
                else ++s_dash.back_press_count;
                hold_back_ms = -1;
            }
            if (!ok && prev_ok && armed_ok) {
                armed_ok = false;
                if (hold_ok_ms >= kLongPressThresholdMs) ++s_dash.long_ok_press_count;
                else ++s_dash.ok_press_count;
                hold_ok_ms = -1;
            }
            dashboard_unlock();
            prev_measure = measure;
            prev_back = back;
            prev_ok = ok;
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static void dashboard_action_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    bool continuous_started = false;
    bool continuous_fast_20hz = false;
    bool previous_requested = false;
    int64_t last_start_us = 0;
    dashboard_laser_stop_continuous();
    dashboard_log_event("laser", "idle at boot; laser disabled outside measurement pages");
    ESP_LOGI("laser_mode", "boot/menu: laser OFF");
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        SingleDistanceAction timed_action = SingleDistanceAction::NONE;
        dashboard_lock();
        timed_action = s_single_distance.tick(now_us);
        dashboard_unlock();
        if (timed_action != SingleDistanceAction::NONE) {
            single_distance_apply_action(timed_action);
            dashboard_log_event("single", "aim timeout; laser off");
        }
        const bool requested = s_laser_active_requested.load(std::memory_order_acquire);
        dashboard_lock();
        const bool room_scan_active_now = s_room.active;
        dashboard_unlock();
        if (room_scan_active_now) room_scan_update_attitude();
        if (continuous_started && requested && room_scan_active_now != continuous_fast_20hz) {
            dashboard_laser_stop_continuous();
            continuous_started = false;
            last_start_us = 0;
        }
        if (!requested) {
            if (continuous_started || previous_requested ||
                s_laser_continuous_active.load(std::memory_order_acquire)) {
                const bool stopped = dashboard_laser_stop_continuous();
                dashboard_log_event("laser", stopped
                                                 ? "measurement page exited; iHALT + iLD:0"
                                                 : "measurement page exited; laser stop failed");
                ESP_LOGI("laser_mode", "measurement page exit: laser %s", stopped ? "OFF" : "STOP FAILED");
            }
            continuous_started = false;
            s_laser_measure_request.store(false, std::memory_order_release);
            s_laser_single_request.store(false, std::memory_order_release);
            s_single_distance_measure_request.store(false, std::memory_order_release);
        } else {
            if (!continuous_started && (last_start_us == 0 || now_us - last_start_us >= 3000000)) {
                continuous_started = dashboard_laser_start_continuous(room_scan_active_now);
                continuous_fast_20hz = room_scan_active_now;
                last_start_us = esp_timer_get_time();
                dashboard_lock();
                s_dash.laser_ready = false;
                dashboard_unlock();
                dashboard_log_event("laser", continuous_started
                                                 ? (continuous_fast_20hz
                                                        ? "room scan entered; iLD:1 + iFACM 20Hz"
                                                        : "measurement page entered; iLD:1 + iACM")
                                                 : "measurement page laser start failed");
                ESP_LOGI("laser_mode", "measurement page enter: laser %s",
                         continuous_started ? (continuous_fast_20hz ? "ON / iFACM 20Hz" : "ON / iACM")
                                            : "START FAILED");
                if (!continuous_started) {
                    bool single_failed = false;
                    dashboard_lock();
                    const SingleDistanceSnapshot single = s_single_distance.snapshot();
                    if (single.page_active &&
                        (single.state == SingleDistanceState::AIMING ||
                         single.state == SingleDistanceState::MEASURING)) {
                        single_failed = s_single_distance.measurement_failed(
                            SingleDistanceError::LASER_UNAVAILABLE, static_cast<int32_t>(ESP_FAIL));
                        s_dash.laser_busy = false;
                    }
                    dashboard_unlock();
                    if (single_failed) {
                        s_laser_active_requested.store(false, std::memory_order_release);
                        dashboard_set_error("single measurement laser start failed; check module and UART");
                    }
                }
            }
            if (continuous_started) {
                dashboard_laser_poll_continuous();
                dashboard_lock();
                const bool room_scan_active = s_room.active;
                dashboard_unlock();
                if (room_scan_active) room_scan_append_sample();
                const int64_t poll_us = esp_timer_get_time();
                if (poll_us - last_start_us > 3000000 &&
                    (s_laser_latest_us <= 0 || poll_us - s_laser_latest_us > 1500000)) {
                    continuous_started = dashboard_laser_start_continuous(room_scan_active_now);
                    continuous_fast_20hz = room_scan_active_now;
                    last_start_us = poll_us;
                    dashboard_lock();
                    s_dash.laser_ready = false;
                    dashboard_unlock();
                    dashboard_log_event("laser", continuous_started
                                                     ? (continuous_fast_20hz
                                                            ? "iFACM 20Hz restarted after stale stream"
                                                            : "iACM restarted after stale stream")
                                                     : "iACM restart failed");
                }
                if (s_laser_measure_request.exchange(false, std::memory_order_acq_rel)) {
                    dashboard_laser_measure_once();
                }
                if (s_laser_single_request.exchange(false, std::memory_order_acq_rel)) {
                    dashboard_laser_measure_single();
                }
                if (s_single_distance_measure_request.exchange(false, std::memory_order_acq_rel)) {
                    dashboard_single_distance_measure();
                }
                if (s_p2p_measure_request.exchange(false, std::memory_order_acq_rel)) {
                    dashboard_p2p_measure(s_p2p_measure_target.exchange(-1, std::memory_order_acq_rel));
                }
            }
        }
        if (s_single_distance_save_request.exchange(false, std::memory_order_acq_rel)) {
            dashboard_single_distance_save();
        }
        if (s_p2p_save_request.exchange(false, std::memory_order_acq_rel)) {
            dashboard_p2p_save();
        }
        if (s_photo_request.exchange(false, std::memory_order_acq_rel)) {
            std::string path;
            dashboard_capture_photo(&path, "dataset", true);
        }
        if (s_web_toggle_request) {
            s_web_toggle_request = false;
            dashboard_toggle_web();
        }
        // 解除绑定:后台执行(不在 LVGL 上下文里切 WiFi,避免推流/上传
        // 期间 esp_wifi_stop 长时间临界区触发中断看门狗导致重启)
        if (s_pc_unbind_request.exchange(false, std::memory_order_acq_rel)) {
            cmd_pc_unbind();
        }
        previous_requested = requested;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    }
}

static void dashboard_bno_task(void *)
{
    esp_err_t err = i2c_init_once();
    if (err != ESP_OK) {
        dashboard_set_error("BNO086 I2C init failed: " + esp_err_str(err));
        vTaskDelete(nullptr);
    }
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_BNO086_INT);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&cfg);
    if (!bno_select_i2c_addr()) {
        dashboard_set_error("BNO086 0x4A/0x4B not found");
        vTaskDelete(nullptr);
    }

    std::fill(s_bno_tx_seq.begin(), s_bno_tx_seq.end(), 0);
    bno_soft_reset();
    BnoTestState drain_state;
    bno_drain_packets(drain_state, 700, false);

    uint8_t product_req[2] = {BNO_RPT_PRODUCT_ID_REQ, 0x00};
    bno_write_packet(BNO_CH_CONTROL, product_req, sizeof(product_req));
    // The magnetically referenced rotation vector drives both the level
    // display and P2P geometry at 20 Hz.  Raw sensors remain at 10 Hz for
    // diagnostics and calibration.  This is IMU-only fusion; camera frames
    // are not part of the attitude solution.
    bno_enable_feature(BNO_RPT_ACCEL, 100000);
    vTaskDelay(pdMS_TO_TICKS(40));
    bno_enable_feature(BNO_RPT_GYRO, 100000);
    vTaskDelay(pdMS_TO_TICKS(40));
    bno_enable_feature(BNO_RPT_MAG, 100000);
    vTaskDelay(pdMS_TO_TICKS(40));
    bno_enable_feature(BNO_RPT_LINEAR_ACCEL, 50000);
    vTaskDelay(pdMS_TO_TICKS(40));
    bno_enable_feature(BNO_RPT_ROTATION_VECTOR, 50000);
    vTaskDelay(pdMS_TO_TICKS(40));
    bno_enable_feature(BNO_RPT_GAME_RV, 20000);

    int64_t last_log = 0;
    while (true) {
        BnoPacket pkt;
        err = bno_read_packet_wait(pkt, 80);
        if (err == ESP_OK) {
            dashboard_bno_process_packet(pkt);
        }
        int64_t now = esp_timer_get_time();
        if (now - last_log > 100000) {
            dashboard_bno_log_snapshot();
            last_log = now;
        }
        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND) {
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            // Let lower-priority touch polling acquire the shared I2C mutex
            // even while the BNO interrupt remains continuously asserted.
            vTaskDelay(1);
        }
    }
}

static uint8_t battery_percent_from_voltage(float voltage)
{
    // User-requested voltage-range indicator rather than a chemistry/SOC
    // model: 3.30 V is empty and 4.20 V is full.
    static constexpr float kEmptyVoltage = 3.30f;
    static constexpr float kFullVoltage = 4.20f;
    const float fraction = std::clamp((voltage - kEmptyVoltage) /
                                      (kFullVoltage - kEmptyVoltage), 0.0f, 1.0f);
    return static_cast<uint8_t>(std::lround(fraction * 100.0f));
}

static esp_err_t battery_adc_init_once()
{
    if (!s_battery_adc_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_battery_adc_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_battery_adc) {
        xSemaphoreGive(s_battery_adc_mutex);
        return ESP_OK;
    }

    esp_err_t err = adc_oneshot_io_to_channel(PIN_BAT_ADC, &s_battery_adc_unit, &s_battery_adc_channel);
    if (err != ESP_OK) {
        xSemaphoreGive(s_battery_adc_mutex);
        return err;
    }

    adc_oneshot_unit_init_cfg_t init = {};
    init.unit_id = s_battery_adc_unit;
    err = adc_oneshot_new_unit(&init, &s_battery_adc);
    if (err != ESP_OK) {
        xSemaphoreGive(s_battery_adc_mutex);
        return err;
    }

    adc_oneshot_chan_cfg_t chan = {};
    chan.bitwidth = ADC_BITWIDTH_DEFAULT;
    chan.atten = ADC_ATTEN_DB_12;
    err = adc_oneshot_config_channel(s_battery_adc, s_battery_adc_channel, &chan);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(s_battery_adc);
        s_battery_adc = nullptr;
        xSemaphoreGive(s_battery_adc_mutex);
        return err;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cal_cfg = {};
    cal_cfg.unit_id = s_battery_adc_unit;
    cal_cfg.chan = s_battery_adc_channel;
    cal_cfg.atten = ADC_ATTEN_DB_12;
    cal_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_battery_adc_cali);
#endif
    xSemaphoreGive(s_battery_adc_mutex);
    return ESP_OK;
}

static void dashboard_battery_task(void *)
{
    if (battery_adc_init_once() != ESP_OK) { vTaskDelete(nullptr); return; }
    float filtered = 0.0f;
    while (true) {
        int raw_sum = 0, mv_sum = 0, valid = 0;
        if (xSemaphoreTake(s_battery_adc_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            dashboard_set_error("Battery ADC busy");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        for (int i = 0; i < 32; ++i) {
            int raw = 0;
            if (adc_oneshot_read(s_battery_adc, s_battery_adc_channel, &raw) == ESP_OK) {
                raw_sum += raw;
                int mv = 0;
                if (s_battery_adc_cali && adc_cali_raw_to_voltage(s_battery_adc_cali, raw, &mv) == ESP_OK) mv_sum += mv;
                ++valid;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        xSemaphoreGive(s_battery_adc_mutex);
        if (valid > 0) {
            float vadc = s_battery_adc_cali ? static_cast<float>(mv_sum) / valid / 1000.0f
                                            : static_cast<float>(raw_sum) / valid / 4095.0f * 3.3f;
            float voltage = vadc * BAT_VOLTAGE_SCALE;
            filtered = filtered == 0.0f ? voltage : filtered * 0.8f + voltage * 0.2f;
            dashboard_lock();
            s_dash.battery_adc_raw = static_cast<uint16_t>(std::lround(static_cast<float>(raw_sum) / valid));
            s_dash.battery_adc_voltage = vadc;
            s_dash.battery_voltage = filtered;
            s_dash.battery_percent = battery_percent_from_voltage(filtered);
            s_dash.battery_valid = true;
            dashboard_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void dashboard_init_log_header(const char *filename, const char *header)
{
    if (!s_sd_mounted || !s_sd_log_mutex || !filename || !header) return;
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) return;
    OpenLog *log = nullptr;
    for (auto &candidate : s_open_logs) {
        if (strcmp(candidate.name, filename) == 0) {
            log = &candidate;
            break;
        }
    }
    if (log && !log->file) {
        std::string path = std::string(SD_MOUNT_POINT) + "/" + filename;
        log->file = fopen(path.c_str(), "a+");
        if (log->file) setvbuf(log->file, nullptr, _IOFBF, 4096);
    }
    if (log && log->file) {
        fseek(log->file, 0, SEEK_END);
        if (ftell(log->file) == 0) {
            fputs(header, log->file);
            fflush(log->file);
        }
        log->pending = 0;
    }
    xSemaphoreGive(s_sd_log_mutex);
}

static void dashboard_init_logs()
{
    dashboard_init_log_header("bno086.csv", "t_us,ax,ay,az,gx,gy,gz,qi,qj,qk,qr,mx,my,mz,status\n");
    dashboard_init_log_header("laser.csv", "t_us,distance_mm,source,raw\n");
    dashboard_init_log_header("measure_points.csv", "t_us,point_count,laser_mm,p1_x_m,p1_y_m,p1_z_m,p2_x_m,p2_y_m,p2_z_m,distance_m\n");
    dashboard_init_log_header("measure_sessions.csv", "t_us,session_id,point_count,laser_mm,p1_x_m,p1_y_m,p1_z_m,p2_x_m,p2_y_m,p2_z_m,distance_m\n");
    dashboard_init_log_header("room_points.csv", "t_us,room_session_id,point_index,laser_mm,x_m,y_m,z_m,segment_m,open_perimeter_m,closure_m,area_xy_m2\n");
    dashboard_init_log_header("room_sessions.csv", "t_us,room_session_id,point_count,rotation_deg,valid_count,invalid_count\n");
    dashboard_init_log_header("scan_sessions.csv", "t_us,scan_session_id,point_count,rotation_deg,valid_count,invalid_count\n");
    dashboard_init_log_header("events.csv", "t_us,type,detail\n");
    dashboard_init_log_header("imu_calibration.csv", "t_us,step,label,ax,ay,az,gx,gy,gz,qi,qj,qk,qr,status\n");
}

static OpenLog *find_open_log(const char *filename)
{
    for (auto &log : s_open_logs) {
        if (strcmp(log.name, filename) == 0) return &log;
    }
    return nullptr;
}

static bool cmd_imu_cal_clear()
{
    if (!s_sd_mounted || !s_sd_log_mutex) {
        fail("imu_cal_clear", "SD not mounted");
        return false;
    }
    struct CalFile { const char *name; const char *header; };
    static constexpr CalFile files[] = {
        {"bno086.csv", "t_us,ax,ay,az,gx,gy,gz,qi,qj,qk,qr,mx,my,mz,status\n"},
        {"imu_calibration.csv", "t_us,step,label,ax,ay,az,gx,gy,gz,qi,qj,qk,qr,status\n"},
        {"events.csv", "t_us,type,detail\n"},
    };
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        fail("imu_cal_clear", "SD logger busy");
        return false;
    }
    bool ok = true;
    for (const auto &item : files) {
        OpenLog *log = find_open_log(item.name);
        if (log && log->file) {
            fflush(log->file);
            fclose(log->file);
            log->file = nullptr;
            log->pending = 0;
        }
        std::string path = std::string(SD_MOUNT_POINT) + "/" + item.name;
        FILE *f = fopen(path.c_str(), "w");
        if (!f || fputs(item.header, f) < 0) ok = false;
        if (f) fclose(f);
    }
    xSemaphoreGive(s_sd_log_mutex);
    ok ? pass("imu_cal_clear", "bno086.csv, imu_calibration.csv and events.csv reset")
       : fail("imu_cal_clear", "one or more files could not be reset");
    return ok;
}

static bool dump_calibration_file(const char *filename)
{
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    OpenLog *log = find_open_log(filename);
    if (log && log->file) {
        fflush(log->file);
        fclose(log->file);
        log->file = nullptr;
        log->pending = 0;
    }
    std::string path = std::string(SD_MOUNT_POINT) + "/" + filename;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) {
        xSemaphoreGive(s_sd_log_mutex);
        printf("=== FILE ERROR %s errno=%d ===\n", filename, errno);
        return false;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    printf("=== FILE BEGIN %s SIZE=%ld ===\n", filename, size);
    char buffer[512];
    size_t n = 0;
    while ((n = fread(buffer, 1, sizeof(buffer), f)) > 0) fwrite(buffer, 1, n, stdout);
    fclose(f);
    printf("\n=== FILE END %s ===\n", filename);
    fflush(stdout);
    xSemaphoreGive(s_sd_log_mutex);
    return true;
}

static bool cmd_imu_cal_dump()
{
    if (!s_sd_mounted || !s_sd_log_mutex) {
        fail("imu_cal_dump", "SD not mounted");
        return false;
    }
    printf("=== IMU CAL DUMP BEGIN ===\n");
    bool ok = dump_calibration_file("imu_calibration.csv");
    ok = dump_calibration_file("events.csv") && ok;
    ok = dump_calibration_file("bno086.csv") && ok;
    printf("=== IMU CAL DUMP END ===\n");
    ok ? pass("imu_cal_dump", "three calibration files sent")
       : fail("imu_cal_dump", "one or more files failed");
    return ok;
}

static bool cmd_room_dump()
{
    if (!s_sd_mounted || !s_sd_log_mutex) {
        fail("room_dump", "SD not mounted");
        return false;
    }
    printf("=== ROOM DUMP BEGIN ===\n");
    bool ok = dump_calibration_file("room_points.csv");
    ok = dump_calibration_file("room_sessions.csv") && ok;
    printf("=== ROOM DUMP END ===\n");
    ok ? pass("room_dump", "room point and summary files sent")
       : fail("room_dump", "one or more room files failed");
    return ok;
}

static bool cmd_room_export_last()
{
    char file[48] = "";
    const bool ok = room_export_last_from_csv(file, sizeof(file));
    ok ? pass("room_export_last", std::string("last room DXF saved: ") + file)
       : fail("room_export_last", "no valid room session or DXF write failed");
    return ok;
}

static bool cmd_pc_status()
{
    const PcBinding binding = pc_binding_snapshot();
    char pending[PC_UPLOAD_PATH_LEN] = "";
    if (s_pc_upload_mutex && xSemaphoreTake(s_pc_upload_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_pc_upload_count > 0) {
            snprintf(pending, sizeof(pending), "%s", s_pc_pending_upload[s_pc_upload_head]);
        }
        xSemaphoreGive(s_pc_upload_mutex);
    }
    char detail[320];
    snprintf(detail, sizeof(detail),
             "enabled=%u bound=%u pairing=%u connected=%u wifi=%u pc=%s hotspot=%s port=%u "
             "upload_queue=%u/%u head=%s",
             s_pc_link_enabled.load(std::memory_order_acquire) ? 1u : 0u,
             binding.valid ? 1u : 0u,
             s_pc_pairing_active.load(std::memory_order_acquire) ? 1u : 0u,
             s_pc_link_connected.load(std::memory_order_acquire) ? 1u : 0u,
             s_wifi_started ? 1u : 0u, binding.pc_name, binding.hotspot_ssid,
             static_cast<unsigned>(binding.server_port),
             static_cast<unsigned>(s_pc_upload_count), static_cast<unsigned>(PC_UPLOAD_QUEUE_DEPTH),
             pending[0] ? pending : "-");
    info("pc_status", detail);
    return true;
}

// NVS commit 会临时禁用 flash cache,不能在 PSRAM 栈任务(action_task)里执行。
// cmd_pc_unbind 只置标志,真正的 NVS 清除与重启由 app_main 主循环处理。
static bool cmd_pc_unbind()
{
    s_pc_unbind_request.store(true, std::memory_order_release);
    return true;
}

static bool device_ui_begin_imu_calibration()
{
    DashboardState snap;
    dashboard_lock();
    snap = s_dash;
    dashboard_unlock();
    if (!snap.bno_ready) {
        dashboard_set_error("IMU calibration blocked: BNO086 not ready");
        fail("imu_cal", "BNO086 not ready; calibration not started");
        return false;
    }
    bool ok = cmd_imu_cal_clear();
    if (ok) {
        dashboard_lock();
        s_dash.last_error[0] = '\0';
        dashboard_unlock();
        dashboard_log_event("imu_cal", "session_start");
    }
    else dashboard_set_error("IMU calibration log reset failed");
    return ok;
}

static void device_ui_read_state(DeviceUiState *out)
{
    if (!out) return;
    DashboardState snap;
    SingleDistanceSnapshot single_snap;
    PcBinding binding;
    dashboard_lock();
    snap = s_dash;
    single_snap = s_single_distance.snapshot();
    binding = s_pc_binding;
    dashboard_unlock();
    const int64_t now_us = esp_timer_get_time();
    memset(out, 0, sizeof(*out));
    out->startup_complete = snap.startup_complete;
    out->startup_pass_count = snap.startup_pass_count;
    out->startup_warning_count = snap.startup_warning_count;
    out->startup_fail_count = snap.startup_fail_count;
    out->check_lcd = snap.check_lcd;
    out->check_i2c = snap.check_i2c;
    out->check_input = snap.check_input;
    out->check_sd = snap.check_sd;
    out->check_camera = snap.check_camera;
    out->check_laser_uart = snap.check_laser_uart;
    out->camera_ready = s_camera_http_ready;
    // 设备状态页"激光器"显示硬件 UART 自检结果(而非是否正在测距)
    out->laser_ready = snap.check_laser_uart == DeviceCheckState::PASS;
    out->wifi_ready = s_wifi_started;
    out->web_busy = s_web_busy;
    out->pc_pairing_active = s_pc_pairing_active.load(std::memory_order_acquire);
    out->pc_binding_valid = s_pc_binding_loaded.load(std::memory_order_acquire) && binding.valid;
    out->pc_link_connected = s_pc_link_connected.load(std::memory_order_acquire);
    out->room_upload_state = s_room_upload_state.load(std::memory_order_acquire);
    snprintf(out->pc_name, sizeof(out->pc_name), "%s", binding.pc_name);
    out->sd_ready = snap.sd_ready;
    out->bno_ready = snap.bno_ready;
    // laser_ready 已在上面用硬件 UART 状态赋值(设备状态页),不再用 snap.laser_ready 覆盖
    out->laser_busy = snap.laser_busy;
    out->key_measure = snap.key_measure;
    out->key_back = snap.key_back;
    out->key_ok = snap.key_ok;
    out->laser_mm = snap.laser_mm;
    out->ax = snap.body_ax; out->ay = snap.body_ay; out->az = snap.body_az;
    out->gx = snap.body_gx; out->gy = snap.body_gy; out->gz = snap.body_gz;
    out->qi = snap.body_qi; out->qj = snap.body_qj; out->qk = snap.body_qk; out->qr = snap.body_qr;
    out->path_x = snap.path_x; out->path_y = snap.path_y; out->path_z = snap.path_z;
    out->fusion_vx = snap.fusion_vx; out->fusion_vy = snap.fusion_vy;
    out->fusion_yaw = snap.fusion_yaw; out->fusion_pitch = snap.fusion_pitch; out->fusion_roll = snap.fusion_roll;
    out->fusion_confidence = snap.fusion_confidence;
    out->fusion_stationary = snap.fusion_stationary;
    out->measure_point_count = snap.measure_point_count;
    out->measure_session_id = snap.measure_session_id;
    out->room_active = snap.room_active;
    out->room_complete = snap.room_complete;
    out->room_session_id = snap.room_session_id;
    out->room_point_count = snap.room_point_count;
    out->room_last_segment_m = snap.room_last_segment_m;
    out->room_open_perimeter_m = snap.room_open_perimeter_m;
    out->room_closure_m = snap.room_closure_m;
    out->room_area_xy_m2 = snap.room_area_xy_m2;
    out->room_current_angle_deg = snap.room_current_angle_deg;
    out->room_rotation_deg = snap.room_rotation_deg;
    out->room_pitch_deg = snap.room_active ? snap.room_pitch_deg : snap.fusion_pitch * 57.2957795f;
    out->room_valid_count = snap.room_valid_count;
    out->room_invalid_count = snap.room_invalid_count;
    out->room_pose_ready = snap.room_active ? snap.room_pose_ready : snap.bno_ready;
    out->room_motion_risk = snap.room_motion_risk;
    out->room_max_linear_accel_mps2 = snap.room_max_linear_accel_mps2;
    out->room_scan_coverage_complete = snap.room_scan_coverage_complete;
    snprintf(out->room_scan_file, sizeof(out->room_scan_file), "%s", snap.room_scan_file);
    out->room_dxf_saved = snap.room_dxf_saved;
    snprintf(out->room_dxf_file, sizeof(out->room_dxf_file), "%s", snap.room_dxf_file);
    out->point_distance_m = snap.point_distance_m;
    out->point1_x = snap.point1_x; out->point1_y = snap.point1_y; out->point1_z = snap.point1_z;
    out->point2_x = snap.point2_x; out->point2_y = snap.point2_y; out->point2_z = snap.point2_z;
    out->p2p_stage = snap.p2p_stage;
    out->p2p_a_valid = snap.p2p_a_valid; out->p2p_b_valid = snap.p2p_b_valid;
    out->p2p_distance_a_mm = snap.p2p_distance_a_mm;
    out->p2p_distance_b_mm = snap.p2p_distance_b_mm;
    out->p2p_timestamp_a_ms = snap.p2p_timestamp_a_ms;
    out->p2p_timestamp_b_ms = snap.p2p_timestamp_b_ms;
    out->p2p_pitch_a_deg = snap.p2p_pitch_a_deg; out->p2p_roll_a_deg = snap.p2p_roll_a_deg; out->p2p_yaw_a_deg = snap.p2p_yaw_a_deg;
    out->p2p_pitch_b_deg = snap.p2p_pitch_b_deg; out->p2p_roll_b_deg = snap.p2p_roll_b_deg; out->p2p_yaw_b_deg = snap.p2p_yaw_b_deg;
    out->p2p_space_distance_m = snap.p2p_space_distance_m;
    out->p2p_horizontal_distance_m = snap.p2p_horizontal_distance_m;
    out->p2p_height_diff_m = snap.p2p_height_diff_m;
    out->p2p_tripod_mode = snap.p2p_tripod_mode;
    out->p2p_motion_risk = snap.p2p_motion_risk;
    out->p2p_relative_angle_deg = snap.p2p_relative_angle_deg;
    out->p2p_max_linear_accel_mps2 = snap.p2p_max_linear_accel_mps2;
    out->p2p_max_gyro_dps = snap.p2p_max_gyro_dps;
    out->p2p_imu_sync_error_ms = snap.p2p_imu_sync_error_ms;
    snprintf(out->p2p_image_a, sizeof(out->p2p_image_a), "%s", snap.p2p_image_a);
    snprintf(out->p2p_image_b, sizeof(out->p2p_image_b), "%s", snap.p2p_image_b);
    out->battery_adc_raw = snap.battery_adc_raw;
    out->battery_adc_voltage = snap.battery_adc_voltage;
    out->battery_voltage = snap.battery_voltage;
    out->battery_percent = snap.battery_percent;
    out->battery_valid = snap.battery_valid;
    out->bno_count = snap.bno_count;
    out->laser_count = snap.laser_count;
    out->photo_count = snap.photo_count;
    out->last_saved_record_id = snap.last_saved_record_id;
    out->single_state = single_snap.state;
    out->single_error = single_snap.error;
    out->single_selected_reference = single_snap.selected_reference;
    out->single_measurement_reference = single_snap.measurement_reference;
    out->single_aim_deadline_us = single_snap.aim_deadline_us;
    out->single_result_valid = single_snap.result_valid;
    out->single_result = single_snap.result;
    out->single_history_count = single_snap.history_count;
    memcpy(out->single_history, single_snap.history, sizeof(out->single_history));
    out->measurement_record_count = snap.measurement_record_count;
    out->measurement_record_visible = snap.measurement_record_visible;
    out->setting_auto_save_single = snap.setting_auto_save_single;
    out->setting_photo_on_measure = snap.setting_photo_on_measure;
    out->setting_distance_unit = snap.setting_distance_unit;
    out->measure_press_count = snap.measure_press_count;
    out->back_press_count = snap.back_press_count;
    out->ok_press_count = snap.ok_press_count;
    out->long_measure_press_count = snap.long_measure_press_count;
    out->long_back_press_count = snap.long_back_press_count;
    out->long_ok_press_count = snap.long_ok_press_count;
    out->uptime_us = now_us;
    snprintf(out->last_photo, sizeof(out->last_photo), "%.*s",
             static_cast<int>(sizeof(out->last_photo) - 1), snap.last_photo);
    snprintf(out->last_error, sizeof(out->last_error), "%.*s",
             static_cast<int>(sizeof(out->last_error) - 1), snap.last_error);
    dashboard_lock();
    const uint16_t room_count = std::min<uint16_t>(s_room.count, ROOM_MAX_POINTS);
    memcpy(out->room_x, s_room.x, static_cast<size_t>(room_count) * sizeof(float));
    memcpy(out->room_y, s_room.y, static_cast<size_t>(room_count) * sizeof(float));
    memcpy(out->measurement_records, s_measurement_records, sizeof(out->measurement_records));
    dashboard_unlock();
}

static bool device_ui_read_camera(uint16_t *pixels, size_t pixel_count)
{
    constexpr size_t required = CAMERA_JPEG_OUTPUT_PIXEL_COUNT;
    if (!pixels || pixel_count < required || !s_camera_http_ready || !s_camera_mutex) return false;
    if (!s_preview_decoder) return false;
    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
    const uint8_t requested_zoom = std::min<uint8_t>(s_camera_zoom.load(std::memory_order_acquire), 1);
    if (requested_zoom != s_decoder_zoom) {
        // Both zoom modes deliberately use the same 320x240 decoder output;
        // zoom is an explicit centre crop in the LVGL task. Do not tear down
        // and recreate the JPEG handle on a user toggle, which can stall the
        // camera task while the UI is waiting for its next frame.
        s_decoder_zoom = requested_zoom;
        ESP_LOGI("camera", "local preview zoom=%ux (decoder unchanged)",
                 requested_zoom ? 2u : 1u);
    }
    const int64_t capture_begin = esp_timer_get_time();
    camera_fb_t *fb = esp_camera_fb_get();
    const int64_t decode_begin = esp_timer_get_time();
    bool ok = false;
    camera_jpeg_decoded_frame_t decoded = {};
    if (fb && fb->format == PIXFORMAT_JPEG) {
        ok = camera_jpeg_decoder_decode(s_preview_decoder, fb->buf, fb->len,
                                        pixels, pixel_count, &decoded) == ESP_OK;
    }
    if (fb) esp_camera_fb_return(fb);
    xSemaphoreGive(s_camera_mutex);
    if (ok) {
        static uint32_t perf_frames = 0;
        static int64_t perf_capture_us = 0, perf_parse_us = 0;
        static int64_t perf_decode_us = 0, perf_start_us = 0;
        const int64_t now = esp_timer_get_time();
        if (perf_start_us == 0) perf_start_us = capture_begin;
        ++perf_frames;
        perf_capture_us += decode_begin - capture_begin;
        perf_parse_us += decoded.parse_us;
        perf_decode_us += decoded.process_us;
        if (now - perf_start_us >= 5000000) {
            ESP_LOGI("camera_perf", "capture=%.1fms parse=%.1fms SIMD_decode=%.1fms producer=%.1ffps",
                     static_cast<double>(perf_capture_us) / perf_frames / 1000.0,
                     static_cast<double>(perf_parse_us) / perf_frames / 1000.0,
                     static_cast<double>(perf_decode_us) / perf_frames / 1000.0,
                     static_cast<double>(perf_frames) * 1000000.0 / (now - perf_start_us));
            perf_frames = 0; perf_capture_us = 0; perf_parse_us = 0;
            perf_decode_us = 0; perf_start_us = now;
        }
    }
    return ok;
}

static bool device_ui_read_record_photo(uint32_t record_id, uint16_t *pixels,
                                        size_t pixel_count)
{
    if (!pixels || pixel_count < CAMERA_JPEG_OUTPUT_PIXEL_COUNT || !s_sd_mounted) return false;
    MeasurementRecord record;
    if (!s_measurement_store.find(record_id, &record) || !record.camera_frame_saved ||
        record.image_path[0] != '/') return false;
    const std::string full_path = std::string(SD_MOUNT_POINT) + record.image_path;
    FILE *file = fopen(full_path.c_str(), "rb");
    if (!file) return false;
    bool ok = false;
    if (fseek(file, 0, SEEK_END) == 0) {
        const long size = ftell(file);
        if (size > 4 && size <= 1024 * 1024 && fseek(file, 0, SEEK_SET) == 0) {
            auto *jpeg = static_cast<uint8_t *>(heap_caps_malloc(
                static_cast<size_t>(size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (jpeg && fread(jpeg, 1, static_cast<size_t>(size), file) == static_cast<size_t>(size)) {
                camera_jpeg_decoder_config_t config = CAMERA_JPEG_DECODER_DEFAULT_CONFIG();
                camera_jpeg_decoder_t *decoder = nullptr;
                if (camera_jpeg_decoder_create(&config, &decoder) == ESP_OK) {
                    camera_jpeg_decoded_frame_t decoded = {};
                    ok = camera_jpeg_decoder_decode(decoder, jpeg, static_cast<size_t>(size),
                                                    pixels, pixel_count, &decoded) == ESP_OK;
                    camera_jpeg_decoder_destroy(decoder);
                }
            }
            heap_caps_free(jpeg);
        }
    }
    fclose(file);
    return ok;
}

static void device_ui_set_camera_zoom(uint8_t zoom_level)
{
    s_camera_zoom.store(zoom_level ? 1 : 0, std::memory_order_release);
}

static bool device_ui_read_touch(uint16_t *x, uint16_t *y, bool *pressed)
{
    static int64_t last_error_log_us = 0;
    static uint32_t suppressed_errors = 0;
    static bool had_error = false;
    if (!x || !y || !pressed) return false;
    *pressed = false;
    esp_err_t init_err = i2c_init_once();
    if (init_err != ESP_OK) {
        ++suppressed_errors;
        had_error = true;
        const int64_t now = esp_timer_get_time();
        if (now - last_error_log_us >= 2000000) {
            ESP_LOGW("touch", "I2C init failed: %s (%lu attempts)", esp_err_to_name(init_err),
                     static_cast<unsigned long>(suppressed_errors));
            last_error_log_us = now;
            suppressed_errors = 0;
        }
        return false;
    }
    uint8_t data[7] = {};
    esp_err_t err = i2c_read_regs(I2C_ADDR_TOUCH, 0x00, data, sizeof(data));
    if (err != ESP_OK) {
        ++suppressed_errors;
        had_error = true;
        const int64_t now = esp_timer_get_time();
        if (now - last_error_log_us >= 2000000) {
            ESP_LOGW("touch", "read 0x15 failed: %s (%lu attempts)", esp_err_to_name(err),
                     static_cast<unsigned long>(suppressed_errors));
            last_error_log_us = now;
            suppressed_errors = 0;
        }
        return false;
    }
    if (had_error) {
        ESP_LOGI("touch", "controller 0x15 polling recovered");
        had_error = false;
        suppressed_errors = 0;
    }
    const uint8_t fingers = data[2] & 0x0F;
    if (fingers > 1 || data[2] == 0xFF) return true;
    uint16_t px = ((data[3] & 0x0F) << 8) | data[4];
    uint16_t py = ((data[5] & 0x0F) << 8) | data[6];
#if TOUCH_SWAP_XY
    std::swap(px, py);
#endif
#if TOUCH_INVERT_X
    px = LCD_WIDTH - 1 - std::min<uint16_t>(px, LCD_WIDTH - 1);
#endif
#if TOUCH_INVERT_Y
    py = LCD_HEIGHT - 1 - std::min<uint16_t>(py, LCD_HEIGHT - 1);
#endif
    *x = std::min<uint16_t>(px, LCD_WIDTH - 1);
    *y = std::min<uint16_t>(py, LCD_HEIGHT - 1);
    *pressed = fingers == 1;
    static bool was_pressed = false;
    if (*pressed && !was_pressed) {
        ESP_LOGI("touch", "press x=%u y=%u gesture=%u", *x, *y, data[1]);
    }
    was_pressed = *pressed;
    return true;
}

static void device_ui_set_single_page_active(bool active)
{
    SingleDistanceAction action = SingleDistanceAction::NONE;
    if (active) {
        s_single_distance_cancel_request.store(false, std::memory_order_release);
        s_single_distance_measure_request.store(false, std::memory_order_release);
        s_single_distance_save_request.store(false, std::memory_order_release);
        s_laser_active_requested.store(false, std::memory_order_release);
        dashboard_lock();
        s_single_distance.enter(s_camera_http_ready, s_uart_laser_ready, esp_timer_get_time());
        const SingleDistanceSnapshot snap = s_single_distance.snapshot();
        dashboard_unlock();
        dashboard_log_event("single", "page enter state=" +
                                      std::to_string(static_cast<unsigned>(snap.state)));
    } else {
        dashboard_lock();
        action = s_single_distance.exit();
        s_dash.laser_busy = false;
        dashboard_unlock();
        s_single_distance_save_request.store(false, std::memory_order_release);
        s_laser_active_requested.store(false, std::memory_order_release);
        single_distance_apply_action(action);
        dashboard_log_event("single", "page exit; laser off");
    }
    if (s_action_task) xTaskNotifyGive(s_action_task);
}

static void device_ui_single_trigger()
{
    SingleDistanceAction action = SingleDistanceAction::NONE;
    SingleDistanceSnapshot before;
    dashboard_lock();
    before = s_single_distance.snapshot();
    if (before.state == SingleDistanceState::INIT_ERROR) {
        s_single_distance.enter(s_camera_http_ready, s_uart_laser_ready, esp_timer_get_time());
    }
    action = s_single_distance.trigger(esp_timer_get_time());
    if (action == SingleDistanceAction::START_MEASUREMENT) {
        s_dash.laser_busy = true;
        s_dash.laser_mm = -1;
        s_dash.last_error[0] = '\0';
    }
    const SingleDistanceSnapshot after = s_single_distance.snapshot();
    dashboard_unlock();
    single_distance_apply_action(action);
    dashboard_log_event("single", "trigger " + std::to_string(static_cast<unsigned>(before.state)) +
                                      "->" + std::to_string(static_cast<unsigned>(after.state)));
}

static void device_ui_single_cancel()
{
    dashboard_lock();
    const SingleDistanceAction action = s_single_distance.cancel();
    s_dash.laser_busy = false;
    dashboard_unlock();
    single_distance_apply_action(action);
    dashboard_log_event("single", "cancel action=" + std::to_string(static_cast<unsigned>(action)));
}

static void device_ui_single_save()
{
    dashboard_lock();
    const bool requested = s_single_distance.request_save();
    dashboard_unlock();
    if (requested) {
        s_single_distance_save_request.store(true, std::memory_order_release);
        if (s_action_task) xTaskNotifyGive(s_action_task);
        dashboard_log_event("single", "formal save requested");
    }
}

static void device_ui_single_cycle_reference()
{
    dashboard_lock();
    const bool changed = s_single_distance.cycle_reference();
    const DistanceReference reference = s_single_distance.snapshot().selected_reference;
    dashboard_unlock();
    if (changed) {
        dashboard_log_event("single", "reference=" +
                                      std::to_string(static_cast<unsigned>(reference)));
    }
}

static void device_ui_request_single_measure()
{
    s_laser_single_request.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
}

static void device_ui_request_measure()
{
    // P2P/room pages own the aiming laser. Reassert ownership here so a prior
    // diagnostic/emergency stop cannot leave a live button connected to an
    // inactive backend.
    s_laser_active_requested.store(true, std::memory_order_release);
    s_laser_measure_request.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
}
static void device_ui_set_p2p_page_active(bool active)
{
    s_p2p_measure_request.store(false, std::memory_order_release);
    s_p2p_measure_target.store(-1, std::memory_order_release);
    s_laser_active_requested.store(false, std::memory_order_release);
    if (active) {
        char orphan_a[sizeof(s_dash.p2p_image_a)] = {};
        char orphan_b[sizeof(s_dash.p2p_image_b)] = {};
        dashboard_lock();
        if (s_dash.p2p_stage != P2pStage::SAVED) {
            snprintf(orphan_a, sizeof(orphan_a), "%s", s_dash.p2p_image_a);
            snprintf(orphan_b, sizeof(orphan_b), "%s", s_dash.p2p_image_b);
        }
        s_dash.p2p_stage = P2pStage::WAIT_A;
        s_dash.p2p_a_valid = false; s_dash.p2p_b_valid = false;
        s_dash.p2p_distance_a_mm = 0; s_dash.p2p_distance_b_mm = 0;
        s_dash.p2p_timestamp_a_ms = 0; s_dash.p2p_timestamp_b_ms = 0;
        s_dash.p2p_pitch_a_deg = 0.0f; s_dash.p2p_roll_a_deg = 0.0f; s_dash.p2p_yaw_a_deg = 0.0f;
        s_dash.p2p_pitch_b_deg = 0.0f; s_dash.p2p_roll_b_deg = 0.0f; s_dash.p2p_yaw_b_deg = 0.0f;
        s_dash.p2p_ax_m = 0.0f; s_dash.p2p_ay_m = 0.0f; s_dash.p2p_az_m = 0.0f;
        s_dash.p2p_bx_m = 0.0f; s_dash.p2p_by_m = 0.0f; s_dash.p2p_bz_m = 0.0f;
        s_dash.p2p_space_distance_m = -1.0f;
        s_dash.p2p_horizontal_distance_m = -1.0f;
        s_dash.p2p_height_diff_m = 0.0f;
        s_dash.p2p_motion_risk = 0;
        s_dash.p2p_relative_angle_deg = 0.0f;
        s_dash.p2p_max_linear_accel_mps2 = 0.0f;
        s_dash.p2p_accel_deviation_sum = 0.0f;
        s_dash.p2p_accel_sample_count = 0;
        s_dash.p2p_max_gyro_dps = 0.0f;
        s_dash.p2p_imu_sync_error_ms = 0;
        s_dash.p2p_motion_start_us = 0;
        s_dash.p2p_pose_a = {};
        s_dash.p2p_pose_b = {};
        s_dash.p2p_pose_a_uses_game_rv = false;
        s_dash.p2p_image_a[0] = '\0'; s_dash.p2p_image_b[0] = '\0';
        s_dash.measure_point_count = 0; s_dash.point_distance_m = -1.0f;
        dashboard_unlock();
        if (orphan_a[0] == '/') unlink(orphan_a);
        if (orphan_b[0] == '/') unlink(orphan_b);
    }
    if (s_action_task) xTaskNotifyGive(s_action_task);
}

static void device_ui_p2p_remeasure(uint8_t point_index)
{
    if (point_index > 1) return;
    char orphan[sizeof(s_dash.p2p_image_a)] = {};
    dashboard_lock();
    if (s_dash.p2p_stage == P2pStage::SAVED) {
        dashboard_unlock();
        return;
    }
    if (point_index == 0) {
        snprintf(orphan, sizeof(orphan), "%s", s_dash.p2p_image_a);
        s_dash.p2p_image_a[0] = '\0';
        s_dash.p2p_a_valid = false;
        s_dash.p2p_pose_a = {};
        s_dash.p2p_pose_a_uses_game_rv = false;
        s_dash.p2p_motion_start_us = 0;
        s_dash.p2p_stage = P2pStage::WAIT_A;
    } else {
        snprintf(orphan, sizeof(orphan), "%s", s_dash.p2p_image_b);
        s_dash.p2p_image_b[0] = '\0';
        s_dash.p2p_b_valid = false;
        s_dash.p2p_pose_b = {};
        s_dash.p2p_stage = P2pStage::WAIT_B;
    }
    p2p_recalculate_locked();
    s_dash.measure_point_count = (s_dash.p2p_a_valid ? 1 : 0) + (s_dash.p2p_b_valid ? 1 : 0);
    dashboard_unlock();
    if (orphan[0] == '/') unlink(orphan);
    s_laser_active_requested.store(false, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
}

static void device_ui_p2p_cycle_mode()
{
    dashboard_lock();
    if (s_dash.p2p_a_valid || s_dash.p2p_b_valid ||
        s_dash.p2p_stage == P2pStage::AIM_A || s_dash.p2p_stage == P2pStage::AIM_B ||
        s_dash.p2p_stage == P2pStage::MEASURE_A || s_dash.p2p_stage == P2pStage::MEASURE_B) {
        dashboard_unlock();
        return;
    }
    s_dash.p2p_tripod_mode = !s_dash.p2p_tripod_mode;
    dashboard_unlock();
}

static void device_ui_p2p_trigger()
{
    dashboard_lock();
    P2pStage stage = s_dash.p2p_stage;
    int target = (stage == P2pStage::WAIT_B || stage == P2pStage::AIM_B || stage == P2pStage::MEASURE_B) ? 1 : 0;
    if (stage == P2pStage::COMPLETE || stage == P2pStage::SAVING || stage == P2pStage::SAVED) {
        dashboard_unlock();
        return;
    }
    const bool aiming = stage == P2pStage::AIM_A || stage == P2pStage::AIM_B;
    if (!aiming) {
        s_dash.p2p_stage = target == 0 ? P2pStage::AIM_A : P2pStage::AIM_B;
        s_dash.laser_busy = false;
        s_dash.last_error[0] = '\0';
        dashboard_unlock();
        s_laser_active_requested.store(true, std::memory_order_release);
    } else {
        s_dash.p2p_stage = target == 0 ? P2pStage::MEASURE_A : P2pStage::MEASURE_B;
        s_dash.laser_busy = true;
        dashboard_unlock();
        s_p2p_measure_target.store(target, std::memory_order_release);
        s_p2p_measure_request.store(true, std::memory_order_release);
    }
    if (s_action_task) xTaskNotifyGive(s_action_task);
}

static void device_ui_p2p_reset() { device_ui_set_p2p_page_active(true); }
static void device_ui_p2p_save()
{
    s_p2p_save_request.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
}
static void device_ui_request_photo()
{
    s_photo_request.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
}
static bool device_ui_delete_measurement_record(uint32_t id)
{
    MeasurementRecord record;
    const bool has_record = s_measurement_store.find(id, &record);
    esp_err_t err = s_measurement_store.erase(id, esp_timer_get_time());
    if (err != ESP_OK) {
        dashboard_set_error("record delete failed: " + std::string(esp_err_to_name(err)));
        return false;
    }
    if (has_record && record.camera_frame_saved && record.image_path[0] == '/') {
        const std::string image = std::string(SD_MOUNT_POINT) + record.image_path;
        if (unlink(image.c_str()) != 0 && errno != ENOENT) {
            dashboard_log_event("photo", "record image cleanup failed id=" + std::to_string(id));
        }
    }
    if (has_record && record.image_path_b[0] == '/') {
        const std::string image_b = std::string(SD_MOUNT_POINT) + record.image_path_b;
        if (unlink(image_b.c_str()) != 0 && errno != ENOENT) {
            dashboard_log_event("photo", "record B image cleanup failed id=" + std::to_string(id));
        }
    }
    publish_measurement_records();
    dashboard_log_event("record", "deleted id=" + std::to_string(id));
    dashboard_lock();
    s_dash.last_error[0] = '\0';
    dashboard_unlock();
    return true;
}

static void device_ui_cycle_setting(uint8_t category)
{
    // 距离单位(mm/cm/m)循环;自动保存/测量时拍照已从设置页移除。
    // NVS 写入交给 nvs_flush 任务异步执行(LVGL 任务内同步擦写会阻塞/崩溃)。
    if (category == 2) {
        dashboard_lock();
        AppSettings settings = s_app_settings;
        switch (settings.distance_unit) {
        case DistanceUnit::MILLIMETRES:
            settings.distance_unit = DistanceUnit::CENTIMETRES;
            break;
        case DistanceUnit::CENTIMETRES:
            settings.distance_unit = DistanceUnit::METRES;
            break;
        case DistanceUnit::METRES:
        default:
            settings.distance_unit = DistanceUnit::MILLIMETRES;
            break;
        }
        s_app_settings = settings;
        dashboard_unlock();
        s_settings_save_pending.store(true, std::memory_order_release);
        publish_app_settings(settings);
        const char *unit_name = settings.distance_unit == DistanceUnit::METRES ? "m" :
                                settings.distance_unit == DistanceUnit::CENTIMETRES ? "cm" : "mm";
        dashboard_log_event("setting", std::string("distance unit: ") + unit_name);
    }
}

static void device_ui_set_laser_active(bool active)
{
    const bool previous = s_laser_active_requested.exchange(active, std::memory_order_acq_rel);
    if (!active) {
        s_laser_measure_request.store(false, std::memory_order_release);
        s_laser_single_request.store(false, std::memory_order_release);
        s_single_distance_measure_request.store(false, std::memory_order_release);
    }
    if (previous != active && s_action_task) xTaskNotifyGive(s_action_task);
}
static void device_ui_request_web_toggle()
{
    if (!s_web_busy) s_web_toggle_request = true;
}

// UI 设置页「解除绑定」:仅置标志,由 action_task 后台执行 NVS 清除与 WiFi 切换,
// 避免在 LVGL 事件上下文里直接切 WiFi 触发看门狗重启。
static bool device_ui_unbind_pc()
{
    s_pc_unbind_request.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    return true;
}

static bool device_ui_confirm_imu_calibration(uint8_t step)
{
    if (step >= std::size(kImuCalibrationLabels)) return false;
    DashboardState snap;
    dashboard_lock();
    snap = s_dash;
    dashboard_unlock();
    char line[360];
    snprintf(line, sizeof(line),
             "%lld,%u,%s,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u\n",
             static_cast<long long>(esp_timer_get_time()), step + 1, kImuCalibrationLabels[step],
             static_cast<double>(snap.ax), static_cast<double>(snap.ay), static_cast<double>(snap.az),
             static_cast<double>(snap.gx), static_cast<double>(snap.gy), static_cast<double>(snap.gz),
             static_cast<double>(snap.qi), static_cast<double>(snap.qj), static_cast<double>(snap.qk),
             static_cast<double>(snap.qr), snap.bno_status);
    if (!snap.bno_ready) {
        dashboard_set_error("IMU calibration paused: BNO086 not ready");
        return false;
    }

    // Expected gravity direction in the raw BNO sensor frame. Calibration has
    // established body X=-sensor Z, body Y=-sensor Y, body Z=-sensor X.
    // Each entry is {raw axis: 0=X/1=Y/2=Z, expected sign}.
    static constexpr int8_t expected_pose[][2] = {
        {0,-1}, {0,+1}, {2,-1}, {2,+1}, {1,-1}, {1,+1}, {0,-1},
        {0,-1}, {0,-1}, {1,-1}, {0,-1}, {2,+1}, {0,-1},
    };
    const float accel[3] = {snap.ax, snap.ay, snap.az};
    int axis = expected_pose[step][0];
    int sign = expected_pose[step][1];
    bool dominant_ok = accel[axis] * sign >= 7.0f;
    bool cross_ok = true;
    for (int i = 0; i < 3; ++i) {
        if (i != axis && fabsf(accel[i]) > 4.0f) cross_ok = false;
    }
    float gyro_norm = sqrtf(snap.gx * snap.gx + snap.gy * snap.gy + snap.gz * snap.gz);
    if (!dominant_ok || !cross_ok || gyro_norm > 2.0f) {
        char reason[160];
        snprintf(reason, sizeof(reason),
                 "IMU calibration pose invalid step=%u accel=%.2f/%.2f/%.2f gyro_norm=%.2f",
                 step + 1, static_cast<double>(snap.ax), static_cast<double>(snap.ay),
                 static_cast<double>(snap.az), static_cast<double>(gyro_norm));
        dashboard_set_error(reason);
        dashboard_log_event("imu_cal_reject", reason);
        return false;
    }
    bool written = dashboard_log_csv("imu_calibration.csv", line);
    if (written) {
        dashboard_lock();
        s_dash.last_error[0] = '\0';
        dashboard_unlock();
        dashboard_log_event("imu_cal", std::to_string(step + 1) + ":" + kImuCalibrationLabels[step]);
    } else {
        dashboard_set_error("IMU calibration marker write failed");
    }
    return written;
}

static bool device_ui_finish_imu_calibration()
{
    dashboard_log_event("imu_cal", "session_complete");
    if (!s_sd_log_mutex || xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        dashboard_set_error("IMU calibration final sync failed: logger busy");
        return false;
    }
    bool ok = true;
    static constexpr const char *files[] = {"bno086.csv", "imu_calibration.csv", "events.csv"};
    for (const char *name : files) {
        OpenLog *log = find_open_log(name);
        if (log && log->file) {
            if (fflush(log->file) != 0) ok = false;
            if (fclose(log->file) != 0) ok = false;
            log->file = nullptr;
            log->pending = 0;
        }
    }
    xSemaphoreGive(s_sd_log_mutex);
    if (!ok) dashboard_set_error("IMU calibration final file close failed");
    return ok;
}

static void device_ui_start_once()
{
    DeviceUiCallbacks callbacks = {};
    callbacks.read_state = device_ui_read_state;
    callbacks.read_camera_rgb565 = device_ui_read_camera;
    callbacks.read_record_photo_rgb565 = device_ui_read_record_photo;
    callbacks.read_touch = device_ui_read_touch;
    callbacks.set_camera_zoom = device_ui_set_camera_zoom;
    callbacks.set_single_page_active = device_ui_set_single_page_active;
    callbacks.single_trigger = device_ui_single_trigger;
    callbacks.single_cancel = device_ui_single_cancel;
    callbacks.single_save = device_ui_single_save;
    callbacks.single_cycle_reference = device_ui_single_cycle_reference;
    callbacks.request_single_measure = device_ui_request_single_measure;
    callbacks.request_measure = device_ui_request_measure;
    callbacks.set_p2p_page_active = device_ui_set_p2p_page_active;
    callbacks.p2p_trigger = device_ui_p2p_trigger;
    callbacks.p2p_remeasure = device_ui_p2p_remeasure;
    callbacks.p2p_cycle_mode = device_ui_p2p_cycle_mode;
    callbacks.p2p_reset = device_ui_p2p_reset;
    callbacks.p2p_save = device_ui_p2p_save;
    callbacks.request_photo = device_ui_request_photo;
    callbacks.request_web_toggle = device_ui_request_web_toggle;
    callbacks.unbind_pc = device_ui_unbind_pc;
    callbacks.delete_measurement_record = device_ui_delete_measurement_record;
    callbacks.cycle_setting = device_ui_cycle_setting;
    callbacks.set_laser_active = device_ui_set_laser_active;
    callbacks.begin_imu_calibration = device_ui_begin_imu_calibration;
    callbacks.confirm_imu_calibration = device_ui_confirm_imu_calibration;
    callbacks.finish_imu_calibration = device_ui_finish_imu_calibration;
    callbacks.begin_room_survey = room_survey_begin;
    callbacks.finish_room_survey = room_survey_finish;
    callbacks.undo_room_survey = room_survey_undo;
    callbacks.cancel_room_survey = room_survey_cancel;
    esp_err_t err = device_ui_start(&callbacks);
    if (err == ESP_OK) pass("device_ui", "menu and live pages started on 240x284 LCD");
    else fail("device_ui", "start failed: " + esp_err_str(err));
}

static void sensor_runtime_start()
{
    if (!s_dash_mutex) {
        s_dash_mutex = xSemaphoreCreateMutex();
    }
    if (!s_laser_mutex) {
        s_laser_mutex = xSemaphoreCreateMutex();
    }
    if (!s_sd_log_mutex) {
        s_sd_log_mutex = xSemaphoreCreateMutex();
    }
    if (!s_battery_adc_mutex) {
        s_battery_adc_mutex = xSemaphoreCreateMutex();
    }
    dashboard_lock();
    if (s_dash.runtime_started) {
        dashboard_unlock();
        return;
    }
    s_dash.runtime_started = true;
    dashboard_unlock();
    if (s_fusion.begin() != ESP_OK) {
        dashboard_set_error("fusion mutex allocation failed");
    }
    if (!measurement_reserve_session_ids_at_boot()) {
        dashboard_set_error("measurement session ID reservation failed");
    }
    if (!room_reserve_session_ids_at_boot()) {
        dashboard_set_error("room session ID reservation failed");
    }
    load_app_settings();

    // 恢复最近一次 LMP1 同步的日历时间(重启后先有近似时间,重连后重新校准)
    int64_t synced_utc_ms = 0;
    int16_t synced_tz_min = 0;
    if (pc_time_sync_load(&synced_utc_ms, &synced_tz_min)) {
        pc_time_sync_apply(synced_utc_ms, synced_tz_min);
    }

    if (!s_sd_mounted) {
        cmd_test_sd();
    }
    dashboard_lock();
    s_dash.sd_ready = s_sd_mounted;
    s_dash.check_sd = s_sd_mounted ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    dashboard_unlock();
    dashboard_init_logs();
    if (s_sd_mounted) {
        esp_err_t record_err = s_measurement_store.begin(SD_MOUNT_POINT);
        if (record_err == ESP_OK) {
            publish_measurement_records();
        } else {
            dashboard_lock();
            s_dash.check_sd = DeviceCheckState::FAIL;
            dashboard_unlock();
            dashboard_set_error("measurement record store failed: " +
                                std::string(esp_err_to_name(record_err)));
        }
    }

    const esp_err_t runtime_i2c_err = i2c_init_once();
    I2cLineDiag runtime_i2c;
    runtime_i2c.idle_sda = gpio_get_level(PIN_I2C_SDA);
    runtime_i2c.idle_scl = gpio_get_level(PIN_I2C_SCL);
    bool i2c_runtime_ok = runtime_i2c_err == ESP_OK &&
                          runtime_i2c.idle_sda == 1 && runtime_i2c.idle_scl == 1;
    if (!i2c_runtime_ok) {
        std::string detail = "I2C runtime devices skipped: " + i2c_line_detail(runtime_i2c);
        dashboard_set_error(detail);
        dashboard_log_event("i2c", detail);
    }
    const bool input_controller_ok = i2c_runtime_ok && i2c_probe(PCA9557_ADDR);
    dashboard_lock();
    s_dash.check_i2c = i2c_runtime_ok ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    s_dash.check_input = input_controller_ok ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    dashboard_unlock();
    if (!input_controller_ok) {
        dashboard_set_error("startup input check failed: PCA9557 not responding");
    }

    const esp_err_t laser_uart_err = uart_init_once(LASER_UART_NUM, PIN_LASER_TX, PIN_LASER_RX,
                                                     LASER_BAUD, &s_uart_laser_ready);
    dashboard_lock();
    s_dash.check_laser_uart = laser_uart_err == ESP_OK ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    dashboard_unlock();
    if (laser_uart_err != ESP_OK) {
        dashboard_set_error("startup laser UART failed: " + std::string(esp_err_to_name(laser_uart_err)));
    } else {
        // Stop the emitter immediately after UART setup.  The action task keeps
        // its delayed stop as a second safety net, but normal boot no longer
        // waits for that task's 500 ms startup delay.
        const bool laser_off = dashboard_laser_stop_continuous(false);
        dashboard_log_event("laser", laser_off
            ? "boot initialization; iHALT + iLD:0"
            : "boot initialization; laser stop failed");
        ESP_LOGI("laser_mode", "boot initialization: laser %s", laser_off ? "OFF" : "STOP FAILED");
        if (!laser_off) {
            dashboard_set_error("startup laser disable command failed");
        }
    }

    if (i2c_runtime_ok && !s_key_task) {
        xTaskCreateWithCaps(dashboard_key_task, "dash_keys", 4096, nullptr, 8, &s_key_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_action_task) {
        // Single-distance save/measure actions copy a 20-result session
        // snapshot and may enter FATFS, so retain ample PSRAM stack headroom.
        xTaskCreateWithCaps(dashboard_action_task, "dash_action", 32768, nullptr, 7, &s_action_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_battery_task) {
        xTaskCreateWithCaps(dashboard_battery_task, "dash_battery", 4096, nullptr, 4, &s_battery_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (i2c_runtime_ok && !s_bno_task) {
        xTaskCreateWithCaps(dashboard_bno_task, "dash_bno", 6144, nullptr, 7, &s_bno_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    // UI 先启动:此时 startup_complete 仍为 false,LVGL 显示启动自检页;
    // 自检完成后置 true,LVGL 主循环自动切到主菜单。
    device_ui_start_once();
    finalize_startup_checks();
    pc_link_start_once();
    // 设备文件管理 HTTP 服务(仅文件接口,STA 模式下可供 PC 代理访问)
    {
        const esp_err_t file_server_err = device_file_server_start();
        if (file_server_err != ESP_OK) {
            dashboard_set_error("device file server start failed: " +
                                std::string(esp_err_to_name(file_server_err)));
        }
    }
    // NVS 待保存泵:内部 RAM 栈,与串口输入解耦
    if (!s_nvs_flush_task) {
        if (xTaskCreate(nvs_flush_task, "nvs_flush", 4096, nullptr, 3, &s_nvs_flush_task) != pdPASS) {
            dashboard_set_error("NVS flush task allocation failed");
        }
    }
    dashboard_log_event("runtime", "started");
}

static bool cmd_test_bat_adc()
{
    esp_err_t err = battery_adc_init_once();
    if (err != ESP_OK) {
        fail("test_bat_adc", "shared ADC init failed: " + esp_err_str(err));
        return false;
    }
    bool cal_ok = s_battery_adc_cali != nullptr;
    int raw_sum = 0;
    int mv_sum = 0;
    int valid = 0;
    if (xSemaphoreTake(s_battery_adc_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        fail("test_bat_adc", "shared ADC busy timeout");
        return false;
    }
    for (int i = 0; i < 100; ++i) {
        int raw = 0;
        if (adc_oneshot_read(s_battery_adc, s_battery_adc_channel, &raw) == ESP_OK) {
            raw_sum += raw;
            int mv = 0;
            if (cal_ok && adc_cali_raw_to_voltage(s_battery_adc_cali, raw, &mv) == ESP_OK) {
                mv_sum += mv;
            }
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    xSemaphoreGive(s_battery_adc_mutex);
    if (valid == 0) {
        fail("test_bat_adc", "no valid ADC samples");
        return false;
    }
    float raw_avg = static_cast<float>(raw_sum) / valid;
    float vadc = cal_ok ? (static_cast<float>(mv_sum) / valid / 1000.0f) : (raw_avg / 4095.0f * 3.3f);
    float vbat = vadc * BAT_VOLTAGE_SCALE;
    char b[160];
    snprintf(b, sizeof(b), "gpio=%d unit=%d channel=%d raw_avg=%.1f vadc=%.3fV vbat=%.3fV cal=%s ratio=%.3f",
             PIN_BAT_ADC, s_battery_adc_unit, s_battery_adc_channel, raw_avg, vadc, vbat,
             cal_ok ? "yes" : "no", BAT_VOLTAGE_SCALE);
    pass("test_bat_adc", b);
    return true;
}

static bool cmd_runtime_status()
{
    DashboardState snap;
    dashboard_lock();
    snap = s_dash;
    dashboard_unlock();
    const bool touch_ok = i2c_probe(I2C_ADDR_TOUCH);
    const bool pca_ok = i2c_probe(PCA9557_ADDR);
    const float mag_uT = std::sqrt(snap.mx * snap.mx + snap.my * snap.my + snap.mz * snap.mz);
    const bool mag_ok = std::isfinite(mag_uT) && mag_uT > 1.0f && mag_uT < 5000.0f;
    char detail[448];
    snprintf(detail, sizeof(detail),
             "camera=%u i2c=%u input=%u bno_ready=%u bno_count=%lu "
             "yaw_deg=%.2f pitch_deg=%.2f roll_deg=%.2f mag_uT=%.2f heading_conf=%.2f "
             "touch_addr=%u pca_addr=%u game_rv_count=%lu p2p_mode=%s p2p_risk=%u p2p_sync_ms=%u",
             s_camera_http_ready ? 1u : 0u,
             snap.check_i2c == DeviceCheckState::PASS ? 1u : 0u,
             snap.check_input == DeviceCheckState::PASS ? 1u : 0u,
             snap.bno_ready ? 1u : 0u,
             static_cast<unsigned long>(snap.bno_count),
             static_cast<double>(snap.fusion_yaw * 57.2957795f),
             static_cast<double>(snap.fusion_pitch * 57.2957795f),
             static_cast<double>(snap.fusion_roll * 57.2957795f),
             static_cast<double>(mag_uT),
             static_cast<double>(snap.fusion_confidence),
             touch_ok ? 1u : 0u,
             pca_ok ? 1u : 0u,
             static_cast<unsigned long>(snap.game_rv_count),
             snap.p2p_tripod_mode ? "tripod" : "handheld",
             static_cast<unsigned>(snap.p2p_motion_risk),
             static_cast<unsigned>(snap.p2p_imu_sync_error_ms));
    const bool ok = s_camera_http_ready && snap.bno_ready && snap.bno_count > 0 &&
                    mag_ok &&
                    snap.check_i2c == DeviceCheckState::PASS &&
                    snap.check_input == DeviceCheckState::PASS &&
                    touch_ok && pca_ok;
    ok ? pass("runtime_status", detail) : fail("runtime_status", detail);
    return ok;
}

static esp_err_t wifi_base_init_once_internal()
{
    if (s_wifi_ready) {
        return ESP_OK;
    }
    log_memory("before wifi init");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, "wifi", "nvs");
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_LOGI("heap", "wifi buffers rx_static=%d rx_dynamic=%d tx_static=%d tx_dynamic=%d mgmt=%d",
             cfg.static_rx_buf_num, cfg.dynamic_rx_buf_num, cfg.static_tx_buf_num,
             cfg.dynamic_tx_buf_num, cfg.rx_mgmt_buf_num);
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), "wifi", "wifi init");
    // PC binding is persisted by the application. Keep the Wi-Fi driver's
    // runtime configuration in RAM so later AP/STA changes never write flash
    // from the large pc_link task, whose stack intentionally lives in PSRAM.
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        esp_wifi_deinit();
        return err;
    }
    log_memory("after wifi init");
    if (!wifi_memory_headroom_ok()) {
        ESP_LOGE("heap", "Wi-Fi refused: require internal_free >= %u and largest >= %u",
                 static_cast<unsigned>(WIFI_MIN_INTERNAL_FREE),
                 static_cast<unsigned>(WIFI_MIN_INTERNAL_LARGEST));
        esp_wifi_deinit();
        return ESP_ERR_NO_MEM;
    }
    s_wifi_ready = true;
    return ESP_OK;
}

static esp_err_t wifi_stop_if_started_internal()
{
    if (!s_wifi_started) {
        return ESP_OK;
    }
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
        return err;
    }
    s_wifi_started = false;
    return ESP_OK;
}

static esp_err_t wifi_control_dispatch(const WifiControlRequest &request)
{
    switch (request.operation) {
    case WifiControlOperation::INIT:
        return wifi_base_init_once_internal();
    case WifiControlOperation::STOP:
        return wifi_stop_if_started_internal();
    case WifiControlOperation::CONFIGURE_AND_START: {
        ESP_RETURN_ON_ERROR(wifi_base_init_once_internal(), "wifi_ctrl", "base init");
        ESP_RETURN_ON_ERROR(wifi_stop_if_started_internal(), "wifi_ctrl", "stop");
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(request.mode), "wifi_ctrl", "set mode");
        if (request.has_config) {
            wifi_config_t config = request.config;
            ESP_RETURN_ON_ERROR(esp_wifi_set_config(request.interface, &config),
                                "wifi_ctrl", "set config");
        }
        const esp_err_t err = esp_wifi_start();
        if (err == ESP_OK) s_wifi_started = true;
        return err;
    }
    case WifiControlOperation::CONNECT:
        return esp_wifi_connect();
    case WifiControlOperation::SET_POWER_SAVE:
        return esp_wifi_set_ps(request.power_save);
    case WifiControlOperation::SET_MAX_TX_POWER:
        return esp_wifi_set_max_tx_power(request.max_tx_power);
    case WifiControlOperation::SET_BANDWIDTH:
        return esp_wifi_set_bandwidth(request.interface, request.bandwidth);
    }
    return ESP_ERR_INVALID_ARG;
}

static void wifi_control_task(void *)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Copy before dispatch so a timed-out caller cannot overwrite the
        // request while this internal-stack task is still handling it.
        const WifiControlRequest request = s_wifi_control_request;
        s_wifi_control_result = wifi_control_dispatch(request);
        xSemaphoreGive(s_wifi_control_done);
    }
}

static esp_err_t wifi_control_start_once()
{
    if (s_wifi_control_task) return ESP_OK;
    if (!s_wifi_control_mutex) s_wifi_control_mutex = xSemaphoreCreateMutex();
    if (!s_wifi_control_done) s_wifi_control_done = xSemaphoreCreateBinary();
    if (!s_wifi_control_mutex || !s_wifi_control_done) return ESP_ERR_NO_MEM;
    // Pin the stack capability explicitly: this task is the only place where
    // Wi-Fi APIs that may access NVS/flash are executed.
    if (xTaskCreateWithCaps(wifi_control_task, "wifi_ctrl", 8192, nullptr, 6,
                            &s_wifi_control_task,
                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) != pdPASS) {
        s_wifi_control_task = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t wifi_control_call(const WifiControlRequest &request)
{
    ESP_RETURN_ON_ERROR(wifi_control_start_once(), "wifi_ctrl", "task start");
    if (xTaskGetCurrentTaskHandle() == s_wifi_control_task) {
        return wifi_control_dispatch(request);
    }
    if (xSemaphoreTake(s_wifi_control_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    // Discard a stale completion token after an earlier timed-out caller.
    xSemaphoreTake(s_wifi_control_done, 0);
    s_wifi_control_request = request;
    xTaskNotifyGive(s_wifi_control_task);
    const bool completed = xSemaphoreTake(s_wifi_control_done, pdMS_TO_TICKS(30000)) == pdTRUE;
    const esp_err_t result = completed ? s_wifi_control_result : ESP_ERR_TIMEOUT;
    xSemaphoreGive(s_wifi_control_mutex);
    return result;
}

static esp_err_t wifi_base_init_once()
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::INIT;
    return wifi_control_call(request);
}

static esp_err_t wifi_stop_if_started()
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::STOP;
    return wifi_control_call(request);
}

static esp_err_t wifi_configure_and_start(wifi_mode_t mode, wifi_interface_t interface,
                                          const wifi_config_t *config)
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::CONFIGURE_AND_START;
    request.mode = mode;
    request.interface = interface;
    request.has_config = config != nullptr;
    if (config) request.config = *config;
    return wifi_control_call(request);
}

static esp_err_t wifi_connect_controlled()
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::CONNECT;
    return wifi_control_call(request);
}

static esp_err_t wifi_set_power_save_controlled(wifi_ps_type_t power_save)
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::SET_POWER_SAVE;
    request.power_save = power_save;
    return wifi_control_call(request);
}

static esp_err_t wifi_set_max_tx_power_controlled(int8_t max_tx_power)
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::SET_MAX_TX_POWER;
    request.max_tx_power = max_tx_power;
    return wifi_control_call(request);
}

static esp_err_t wifi_set_bandwidth_controlled(wifi_interface_t interface,
                                               wifi_bandwidth_t bandwidth)
{
    WifiControlRequest request;
    request.operation = WifiControlOperation::SET_BANDWIDTH;
    request.interface = interface;
    request.bandwidth = bandwidth;
    return wifi_control_call(request);
}

static bool pc_binding_load(PcBinding *binding)
{
    if (!binding) return false;
    *binding = {};
    nvs_handle_t handle = 0;
    if (nvs_open(PC_LINK_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
    uint8_t valid = 0;
    size_t ssid_size = sizeof(binding->hotspot_ssid);
    size_t password_size = sizeof(binding->hotspot_password);
    size_t name_size = sizeof(binding->pc_name);
    size_t mac_size = sizeof(binding->station_mac);
    uint16_t port = PC_DEFAULT_SERVER_PORT;
    bool ok = nvs_get_u8(handle, "valid", &valid) == ESP_OK && valid == 1 &&
              nvs_get_str(handle, "ssid", binding->hotspot_ssid, &ssid_size) == ESP_OK &&
              nvs_get_str(handle, "password", binding->hotspot_password, &password_size) == ESP_OK &&
              nvs_get_str(handle, "pc_name", binding->pc_name, &name_size) == ESP_OK;
    nvs_get_u16(handle, "port", &port);
    nvs_get_blob(handle, "station_mac", binding->station_mac, &mac_size);
    nvs_close(handle);
    binding->server_port = port ? port : PC_DEFAULT_SERVER_PORT;
    binding->valid = ok && binding->hotspot_ssid[0] != '\0' &&
                     strlen(binding->hotspot_password) >= 8;
    return binding->valid;
}

static PcBinding pc_binding_snapshot()
{
    PcBinding binding;
    dashboard_lock();
    binding = s_pc_binding;
    dashboard_unlock();
    return binding;
}

static bool pc_binding_write_nvs(const PcBinding &binding)
{
    if (!binding.valid || !binding.hotspot_ssid[0] || strlen(binding.hotspot_password) < 8) return false;
    nvs_handle_t handle = 0;
    if (nvs_open(PC_LINK_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t err = nvs_set_u8(handle, "valid", 1);
    if (err == ESP_OK) err = nvs_set_str(handle, "ssid", binding.hotspot_ssid);
    if (err == ESP_OK) err = nvs_set_str(handle, "password", binding.hotspot_password);
    if (err == ESP_OK) err = nvs_set_str(handle, "pc_name", binding.pc_name);
    if (err == ESP_OK) err = nvs_set_u16(handle, "port", binding.server_port);
    if (err == ESP_OK) err = nvs_set_blob(handle, "station_mac", binding.station_mac,
                                           sizeof(binding.station_mac));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err == ESP_OK;
}

static bool pc_binding_save(const PcBinding &binding)
{
    if (!binding.valid || !binding.hotspot_ssid[0] || strlen(binding.hotspot_password) < 8) {
        return false;
    }
    // The PC networking task has a PSRAM stack. NVS commits temporarily
    // disable the flash cache, so hand the small write to app_main, whose
    // stack is internal RAM, and wait for its explicit result.
    dashboard_lock();
    s_pc_binding_to_save = binding;
    dashboard_unlock();
    s_pc_binding_save_ok.store(false, std::memory_order_release);
    s_pc_binding_save_complete.store(false, std::memory_order_release);
    s_pc_binding_save_pending.store(true, std::memory_order_release);
    for (int attempt = 0; attempt < 60; ++attempt) {
        if (s_pc_binding_save_complete.load(std::memory_order_acquire)) {
            return s_pc_binding_save_ok.load(std::memory_order_acquire);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

static void process_pending_pc_binding_save()
{
    if (!s_pc_binding_save_pending.exchange(false, std::memory_order_acq_rel)) return;
    PcBinding binding;
    dashboard_lock();
    binding = s_pc_binding_to_save;
    dashboard_unlock();
    const bool ok = pc_binding_write_nvs(binding);
    if (ok) {
        dashboard_lock();
        s_pc_binding = binding;
        dashboard_unlock();
        s_pc_binding_loaded.store(true, std::memory_order_release);
        // 新绑定已写入:解除 manual-only(重新允许自动绑定),并清除持久化标记
        s_pairing_manual_only.store(false, std::memory_order_release);
        nvs_handle_t flag_handle = 0;
        if (nvs_open("pc_link_flag", NVS_READWRITE, &flag_handle) == ESP_OK) {
            nvs_set_u8(flag_handle, "manual_only", 0);
            nvs_commit(flag_handle);
            nvs_close(flag_handle);
        }
    }
    s_pc_binding_save_ok.store(ok, std::memory_order_release);
    s_pc_binding_save_complete.store(true, std::memory_order_release);
}

// 在 app_main(内部 RAM 栈)执行解除绑定:NVS 清除 + manual-only 标记 + 重启。
// NVS commit 会临时禁用 flash cache,不能在 PSRAM 栈任务里做。
static void process_pending_pc_unbind()
{
    if (!s_pc_unbind_request.exchange(false, std::memory_order_acq_rel)) return;
    const bool ok = pc_link_clear_binding();
    ok ? pass("pc_unbind", "binding cleared; rebooting into pairing AP")
       : fail("pc_unbind", "failed to clear NVS binding");
    if (!ok) return;
    // 解除绑定后仅接受手动绑定(8766 端口写入),防止电脑自动重连
    // 触发 120 秒自动绑定又把热点关掉。标记持久化,重启后恢复。
    s_pairing_manual_only.store(true, std::memory_order_release);
    nvs_handle_t flag_handle = 0;
    if (nvs_open("pc_link_flag", NVS_READWRITE, &flag_handle) == ESP_OK) {
        nvs_set_u8(flag_handle, "manual_only", 1);
        nvs_commit(flag_handle);
        nvs_close(flag_handle);
    }
    ESP_LOGI("pc_unbind", "restart in 800ms to start pairing AP (manual bind only)");
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

static bool pc_link_clear_binding()
{
    nvs_handle_t handle = 0;
    if (nvs_open(PC_LINK_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t err = nvs_erase_all(handle);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) return false;
    dashboard_lock();
    s_pc_binding = {};
    dashboard_unlock();
    s_pc_binding_loaded.store(true, std::memory_order_release);
    s_pc_link_connected.store(false, std::memory_order_release);
    if (s_pc_link_task) xTaskNotifyGive(s_pc_link_task);
    return true;
}

static bool pc_json_string(const char *json, const char *key, char *out, size_t out_size)
{
    if (!json || !key || !out || out_size == 0) return false;
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return false;
    p = strchr(p + strlen(needle), ':');
    if (!p) return false;
    while (*++p && std::isspace(static_cast<unsigned char>(*p))) {}
    if (*p != '"') return false;
    ++p;
    size_t pos = 0;
    while (*p && *p != '"' && pos + 1 < out_size) {
        if (*p == '\\' && p[1]) {
            ++p;
            if (*p == 'n') out[pos++] = '\n';
            else if (*p == 'r') out[pos++] = '\r';
            else if (*p == 't') out[pos++] = '\t';
            else out[pos++] = *p;
        } else {
            out[pos++] = *p;
        }
        ++p;
    }
    out[pos] = '\0';
    return *p == '"' && pos > 0;
}

static uint16_t pc_json_port(const char *json)
{
    const char *key = strstr(json ? json : "", "\"server_port\"");
    if (!key || !(key = strchr(key, ':'))) return PC_DEFAULT_SERVER_PORT;
    const long value = strtol(key + 1, nullptr, 10);
    return value > 0 && value <= 65535 ? static_cast<uint16_t>(value) : PC_DEFAULT_SERVER_PORT;
}

static bool pc_json_long(const char *json, const char *key, int64_t *out)
{
    if (!json || !key || !out) return false;
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return false;
    p = strchr(p + strlen(needle), ':');
    if (!p) return false;
    char *end = nullptr;
    const long long value = strtoll(p + 1, &end, 10);
    if (!end || end == p + 1) return false;
    *out = static_cast<int64_t>(value);
    return true;
}

// ---------------- LMP1 日历时间同步 ----------------
// PC 在 HELLO 应答中下发 utc_ms 与 tz_min(UTC 偏移分钟),设备 settimeofday。
// NVS 持久化交给内部 RAM 栈的 nvs_flush 任务(Pc_link 任务为 PSRAM 栈,
// NVS 提交期间 Flash cache 短暂禁用,直接在 PSRAM 任务里写会干扰执行)。
static std::atomic<bool> s_time_sync_persist_pending{false};
static int64_t s_time_sync_utc_ms = 0;
static int16_t s_time_sync_tz_min = 480;

static void pc_time_sync_persist(int64_t utc_ms, int16_t tz_min)
{
    s_time_sync_utc_ms = utc_ms;
    s_time_sync_tz_min = tz_min;
    s_time_sync_persist_pending.store(true, std::memory_order_release);
}

static void process_pending_time_sync_save()
{
    if (!s_time_sync_persist_pending.exchange(false, std::memory_order_acq_rel)) return;
    nvs_handle_t handle = 0;
    if (nvs_open("time_sync", NVS_READWRITE, &handle) != ESP_OK) return;
    nvs_set_i64(handle, "utc_ms", s_time_sync_utc_ms);
    nvs_set_i16(handle, "tz_min", s_time_sync_tz_min);
    nvs_commit(handle);
    nvs_close(handle);
}

static bool pc_time_sync_load(int64_t *utc_ms, int16_t *tz_min)
{
    nvs_handle_t handle = 0;
    if (nvs_open("time_sync", NVS_READONLY, &handle) != ESP_OK) return false;
    const esp_err_t e1 = nvs_get_i64(handle, "utc_ms", utc_ms);
    const esp_err_t e2 = nvs_get_i16(handle, "tz_min", tz_min);
    nvs_close(handle);
    return e1 == ESP_OK && e2 == ESP_OK;
}

static void pc_time_sync_apply(int64_t utc_ms, int16_t tz_min)
{
    if (utc_ms <= 1609459200000LL) return;   // 早于 2021 视为无效
    struct timeval tv = {};
    tv.tv_sec = utc_ms / 1000;
    tv.tv_usec = (utc_ms % 1000) * 1000;
    settimeofday(&tv, nullptr);
    // POSIX TZ:offset 为西向分钟,东八区 tz_min=+480 -> "UTC-8"
    const long hours = static_cast<long>(tz_min) / 60;
    const long minutes = static_cast<long>(tz_min) % 60;
    char tz[40];
    if (minutes == 0) {
        snprintf(tz, sizeof(tz), "UTC%+ld", -hours);
    } else {
        snprintf(tz, sizeof(tz), "UTC%+ld:%02ld", -hours, std::labs(minutes));
    }
    setenv("TZ", tz, 1);
    tzset();
    ESP_LOGI("time_sync", "LMP1 UTC synced utc_ms=%lld tz=%s", static_cast<long long>(utc_ms), tz);
}

static esp_err_t pc_start_pairing_ap()
{
    ESP_RETURN_ON_ERROR(wifi_base_init_once(), "pc_pair", "wifi init");
    if (!s_wifi_ap_netif) s_wifi_ap_netif = esp_netif_create_default_wifi_ap();
    wifi_config_t config = {};
    snprintf(reinterpret_cast<char *>(config.ap.ssid), sizeof(config.ap.ssid), "%s", PC_PAIRING_SSID);
    snprintf(reinterpret_cast<char *>(config.ap.password), sizeof(config.ap.password), "%s",
             PC_PAIRING_PASSWORD);
    config.ap.ssid_len = strlen(PC_PAIRING_SSID);
    config.ap.channel = 6;
    config.ap.max_connection = 2;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(wifi_configure_and_start(WIFI_MODE_AP, WIFI_IF_AP, &config),
                        "pc_pair", "AP start");
    ESP_RETURN_ON_ERROR(wifi_set_max_tx_power_controlled(WIFI_AP_MAX_TX_POWER_QDBM),
                        "pc_pair", "AP tx power");
    s_pc_pairing_active.store(true, std::memory_order_release);
    dashboard_log_event("pc_link", std::string("pairing AP started ssid=") + PC_PAIRING_SSID);
    return ESP_OK;
}

static bool pc_accept_binding(int server_socket, int64_t *station_since_us, uint8_t station_mac[6])
{
    wifi_sta_list_t stations = {};
    if (esp_wifi_ap_get_sta_list(&stations) == ESP_OK && stations.num > 0) {
        if (memcmp(station_mac, stations.sta[0].mac, 6) != 0) {
            memcpy(station_mac, stations.sta[0].mac, 6);
            *station_since_us = esp_timer_get_time();
        }
    } else {
        memset(station_mac, 0, 6);
        *station_since_us = 0;
    }

    sockaddr_in peer = {};
    socklen_t peer_len = sizeof(peer);
    int client = accept(server_socket, reinterpret_cast<sockaddr *>(&peer), &peer_len);
    if (client >= 0) {
        timeval timeout = {.tv_sec = 3, .tv_usec = 0};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        char request[1024] = {};
        int received = recv(client, request, sizeof(request) - 1, 0);
        PcBinding binding = {};
        bool parsed = received > 0 && pc_json_string(request, "ssid", binding.hotspot_ssid,
                                                      sizeof(binding.hotspot_ssid)) &&
                      pc_json_string(request, "password", binding.hotspot_password,
                                     sizeof(binding.hotspot_password));
        if (parsed) {
            pc_json_string(request, "pc_name", binding.pc_name, sizeof(binding.pc_name));
            if (!binding.pc_name[0]) snprintf(binding.pc_name, sizeof(binding.pc_name), "room-planner-pc");
            memcpy(binding.station_mac, station_mac, sizeof(binding.station_mac));
            binding.server_port = pc_json_port(request);
            binding.valid = strlen(binding.hotspot_ssid) <= 32 &&
                            strlen(binding.hotspot_password) >= 8 &&
                            strlen(binding.hotspot_password) <= 63;
        }
        const bool saved = parsed && pc_binding_save(binding);
        const char *reply = saved ?
            "{\"ok\":true,\"message\":\"binding_saved\"}\n" :
            "{\"ok\":false,\"error\":\"invalid_or_unsaved_binding\"}\n";
        send(client, reply, strlen(reply), 0);
        shutdown(client, SHUT_RDWR);
        close(client);
        return saved;
    }

    if (*station_since_us > 0 &&
        !s_pairing_manual_only.load(std::memory_order_acquire) &&
        esp_timer_get_time() - *station_since_us >= static_cast<int64_t>(PC_PAIRING_AUTO_BIND_MS) * 1000) {
        PcBinding binding = {};
        binding.valid = true;
        snprintf(binding.hotspot_ssid, sizeof(binding.hotspot_ssid), "%s", PC_DEFAULT_HOTSPOT_SSID);
        snprintf(binding.hotspot_password, sizeof(binding.hotspot_password), "%s", PC_DEFAULT_HOTSPOT_PASSWORD);
        snprintf(binding.pc_name, sizeof(binding.pc_name), "client-%02x%02x%02x", station_mac[3],
                 station_mac[4], station_mac[5]);
        memcpy(binding.station_mac, station_mac, sizeof(binding.station_mac));
        binding.server_port = PC_DEFAULT_SERVER_PORT;
        return pc_binding_save(binding);
    }
    return false;
}

static bool pc_pairing_loop()
{
    int server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_socket < 0) return false;
    int reuse = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    timeval timeout = {.tv_sec = 0, .tv_usec = 250000};
    setsockopt(server_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(PC_PAIRING_PORT);
    bool ready = bind(server_socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0 &&
                 listen(server_socket, 2) == 0;
    int64_t station_since_us = 0;
    uint8_t station_mac[6] = {};
    while (ready && s_pc_link_enabled.load(std::memory_order_acquire) &&
           !pc_binding_snapshot().valid) {
        if (pc_accept_binding(server_socket, &station_since_us, station_mac)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    shutdown(server_socket, SHUT_RDWR);
    close(server_socket);
    s_pc_pairing_active.store(false, std::memory_order_release);
    return pc_binding_snapshot().valid;
}

static bool pc_connect_station(esp_ip4_addr_t *gateway)
{
    if (!gateway) return false;
    ESP_RETURN_ON_FALSE(wifi_base_init_once() == ESP_OK, false, "pc_link", "wifi init failed");
    if (!s_wifi_sta_netif) s_wifi_sta_netif = esp_netif_create_default_wifi_sta();
    const PcBinding binding = pc_binding_snapshot();
    if (!binding.valid) return false;
    wifi_config_t config = {};
    const size_t ssid_length = std::min(strlen(binding.hotspot_ssid), sizeof(config.sta.ssid));
    const size_t password_length = std::min(strlen(binding.hotspot_password),
                                            sizeof(config.sta.password));
    memcpy(config.sta.ssid, binding.hotspot_ssid, ssid_length);
    memcpy(config.sta.password, binding.hotspot_password, password_length);
    config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    if (wifi_configure_and_start(WIFI_MODE_STA, WIFI_IF_STA, &config) != ESP_OK ||
        wifi_set_power_save_controlled(WIFI_PS_NONE) != ESP_OK ||
        wifi_connect_controlled() != ESP_OK) return false;
    for (int attempt = 0; attempt < 150 && s_pc_link_enabled.load(std::memory_order_acquire); ++attempt) {
        wifi_ap_record_t ap = {};
        esp_netif_ip_info_t ip = {};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
            esp_netif_get_ip_info(s_wifi_sta_netif, &ip) == ESP_OK && ip.ip.addr && ip.gw.addr) {
            *gateway = ip.gw;
            ESP_LOGI("pc_link", "PC hotspot connected ssid=%s ip=" IPSTR " gateway=" IPSTR,
                     binding.hotspot_ssid, IP2STR(&ip.ip), IP2STR(&ip.gw));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

static bool pc_send_all(int socket_fd, const void *data, size_t size)
{
    const uint8_t *cursor = static_cast<const uint8_t *>(data);
    while (size > 0) {
        int sent = send(socket_fd, cursor, size, 0);
        if (sent <= 0) return false;
        cursor += sent;
        size -= static_cast<size_t>(sent);
    }
    return true;
}

static bool pc_send_packet(int socket_fd, uint8_t type, const void *data, size_t size)
{
    if (size > PC_LINK_MAX_PACKET) return false;
    uint8_t header[12] = {'L', 'M', 'P', '1', type, 0, 0, 0,
                          static_cast<uint8_t>((size >> 24) & 0xff),
                          static_cast<uint8_t>((size >> 16) & 0xff),
                          static_cast<uint8_t>((size >> 8) & 0xff),
                          static_cast<uint8_t>(size & 0xff)};
    return pc_send_all(socket_fd, header, sizeof(header)) &&
           (size == 0 || pc_send_all(socket_fd, data, size));
}

static bool pc_receive_reply(int socket_fd, char *reply, size_t reply_size)
{
    if (!reply || reply_size < 2) return false;
    size_t used = 0;
    while (used + 1 < reply_size) {
        int count = recv(socket_fd, reply + used, reply_size - used - 1, 0);
        if (count <= 0) return false;
        used += static_cast<size_t>(count);
        reply[used] = '\0';
        if (strchr(reply, '\n')) return true;
    }
    return false;
}

static int pc_open_receiver_socket(esp_ip4_addr_t gateway)
{
    const PcBinding binding = pc_binding_snapshot();
    if (!binding.valid) return -1;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (fd < 0) return -1;
    timeval timeout = {.tv_sec = 3, .tv_usec = 0};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = gateway.addr;
    address.sin_port = htons(binding.server_port);
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
        close(fd);
        return -1;
    }
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char hello[256];
    snprintf(hello, sizeof(hello),
             "{\"device_id\":\"laser-%02x%02x%02x%02x%02x%02x\",\"firmware\":\"esp-idf-lvgl\","
             "\"protocol\":\"LMP1\",\"camera\":\"OV5640-VGA-JPEG\",\"http_port\":80}",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    char reply[256] = {};
    if (!pc_send_packet(fd, 1, hello, strlen(hello)) || !pc_receive_reply(fd, reply, sizeof(reply)) ||
        (strstr(reply, "\"ok\":true") == nullptr && strstr(reply, "\"ok\": true") == nullptr)) {
        close(fd);
        return -1;
    }
    // LMP1 时间同步:PC 在 HELLO 应答中下发 utc_ms/tz_min
    int64_t utc_ms = 0;
    int64_t tz_min = 480;
    if (pc_json_long(reply, "utc_ms", &utc_ms) && pc_json_long(reply, "tz_min", &tz_min)) {
        pc_time_sync_apply(utc_ms, static_cast<int16_t>(tz_min));
        pc_time_sync_persist(utc_ms, static_cast<int16_t>(tz_min));
    } else {
        ESP_LOGI("time_sync", "PC HELLO reply without utc_ms; time not synced");
    }
    return fd;
}

static bool pc_send_telemetry(int socket_fd)
{
    DashboardState snap;
    dashboard_lock();
    snap = s_dash;
    dashboard_unlock();
    const bool room_pose_ready = snap.room_active ? snap.room_pose_ready : snap.bno_ready;
    const float room_pitch_deg = snap.room_active ? snap.room_pitch_deg : snap.fusion_pitch * 57.2957795f;
    char payload[640];
    const int length = snprintf(payload, sizeof(payload),
        "{\"timestamp_us\":%lld,\"distance_mm\":%ld,\"yaw_deg\":%.3f,\"pitch_deg\":%.3f,"
        "\"roll_deg\":%.3f,\"pose_ready\":%s,\"motion_risk\":%u,"
        "\"max_linear_accel_mps2\":%.3f,\"room_active\":%s,\"room_complete\":%s,"
        "\"angle_deg\":%.3f,\"room_pitch_deg\":%.3f,\"valid_count\":%u,"
        "\"invalid_count\":%u,\"scan_file\":\"%s\"}",
        static_cast<long long>(esp_timer_get_time()), static_cast<long>(snap.laser_mm),
        static_cast<double>(snap.fusion_yaw * 57.2957795f),
        static_cast<double>(snap.fusion_pitch * 57.2957795f),
        static_cast<double>(snap.fusion_roll * 57.2957795f),
        room_pose_ready ? "true" : "false", static_cast<unsigned>(snap.room_motion_risk),
        static_cast<double>(snap.room_max_linear_accel_mps2), snap.room_active ? "true" : "false",
        snap.room_complete ? "true" : "false", static_cast<double>(snap.room_current_angle_deg),
        static_cast<double>(room_pitch_deg),
        static_cast<unsigned>(snap.room_valid_count), static_cast<unsigned>(snap.room_invalid_count),
        snap.room_scan_file);
    return length > 0 && length < static_cast<int>(sizeof(payload)) &&
           pc_send_packet(socket_fd, 2, payload, static_cast<size_t>(length));
}

static bool pc_send_camera_frame(int socket_fd)
{
    if (!s_camera_http_ready || !s_camera_mutex ||
        xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return true;
    camera_fb_t *frame = esp_camera_fb_get();
    bool ok = frame && frame->format == PIXFORMAT_JPEG && frame->len > 0 &&
              frame->len <= PC_LINK_MAX_PACKET && pc_send_packet(socket_fd, 3, frame->buf, frame->len);
    if (frame) esp_camera_fb_return(frame);
    xSemaphoreGive(s_camera_mutex);
    return ok;
}

static bool pc_pending_upload_copy(char *path, size_t path_size)
{
    if (!path || path_size == 0 || !s_pc_upload_mutex ||
        xSemaphoreTake(s_pc_upload_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    const bool has = s_pc_upload_count > 0;
    if (has) {
        snprintf(path, path_size, "%s", s_pc_pending_upload[s_pc_upload_head]);
    }
    xSemaphoreGive(s_pc_upload_mutex);
    return has;
}

static void pc_pending_upload_pop()
{
    if (!s_pc_upload_mutex || xSemaphoreTake(s_pc_upload_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    if (s_pc_upload_count > 0) {
        s_pc_pending_upload[s_pc_upload_head][0] = '\0';
        s_pc_upload_head = (s_pc_upload_head + 1) % PC_UPLOAD_QUEUE_DEPTH;
        --s_pc_upload_count;
    }
    xSemaphoreGive(s_pc_upload_mutex);
}

static bool pc_send_file(int socket_fd, const char *relative_path)
{
    if (!relative_path || relative_path[0] != '/' || !s_sd_mounted) return false;
    const std::string full_path = std::string(SD_MOUNT_POINT) + relative_path;
    struct stat file_stat = {};
    if (stat(full_path.c_str(), &file_stat) != 0 || !S_ISREG(file_stat.st_mode) || file_stat.st_size < 0) {
        return false;
    }
    FILE *file = fopen(full_path.c_str(), "rb");
    if (!file) return false;
    uint8_t buffer[4096];
    uint32_t crc = 0;
    size_t count = 0;
    while ((count = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        crc = esp_rom_crc32_le(crc, buffer, count);
    }
    if (ferror(file) || fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return false;
    }
    const char *base = strrchr(relative_path, '/');
    base = base && base[1] ? base + 1 : "scan.csv";
    char metadata[256];
    int metadata_length = snprintf(metadata, sizeof(metadata),
        "{\"name\":\"%s\",\"size\":%llu,\"crc32\":\"%08lx\"}", base,
        static_cast<unsigned long long>(file_stat.st_size), static_cast<unsigned long>(crc));
    bool ok = metadata_length > 0 && metadata_length < static_cast<int>(sizeof(metadata)) &&
              pc_send_packet(socket_fd, 4, metadata, static_cast<size_t>(metadata_length));
    while (ok && (count = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        ok = pc_send_packet(socket_fd, 5, buffer, count);
    }
    if (ferror(file)) ok = false;
    fclose(file);
    static constexpr char end[] = "{}";
    if (ok) ok = pc_send_packet(socket_fd, 6, end, sizeof(end) - 1);
    char reply[256] = {};
    if (ok) ok = pc_receive_reply(socket_fd, reply, sizeof(reply)) &&
                 strstr(reply, "\"ok\":true") != nullptr;
    if (ok) dashboard_log_event("pc_link", std::string("uploaded ") + relative_path);
    return ok;
}

static void pc_link_queue_upload(const char *relative_path, bool drop_when_full)
{
    const bool is_scan = relative_path && strncmp(relative_path, "/scan_", 6) == 0;
    if (!relative_path || relative_path[0] != '/' || !s_pc_upload_mutex ||
        xSemaphoreTake(s_pc_upload_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        if (is_scan) s_room_upload_state.store(RoomUploadState::RETRYING, std::memory_order_release);
        return;
    }
    if (s_pc_upload_count >= PC_UPLOAD_QUEUE_DEPTH) {
        if (!drop_when_full) {
            // 补扫模式:满了就跳过,不丢队首。每次上传成功腾出槽位后,
            // 下一轮补扫会把该文件补入,避免“丢队首→未标记→再入队”活锁。
            xSemaphoreGive(s_pc_upload_mutex);
            return;
        }
        // 新任务优先:队列满时丢最旧(照片仍在 SD,不丢失原始数据)。
        s_pc_pending_upload[s_pc_upload_head][0] = '\0';
        s_pc_upload_head = (s_pc_upload_head + 1) % PC_UPLOAD_QUEUE_DEPTH;
        --s_pc_upload_count;
        ESP_LOGW("pc_link", "upload queue full; dropped oldest pending item");
    }
    snprintf(s_pc_pending_upload[s_pc_upload_tail], PC_UPLOAD_PATH_LEN, "%s", relative_path);
    s_pc_upload_tail = (s_pc_upload_tail + 1) % PC_UPLOAD_QUEUE_DEPTH;
    ++s_pc_upload_count;
    xSemaphoreGive(s_pc_upload_mutex);
    // 只有扫描 CSV 驱动“平面图”页面的上传状态;照片上传保持静默。
    if (is_scan) s_room_upload_state.store(RoomUploadState::QUEUED, std::memory_order_release);
    if (s_pc_link_task) xTaskNotifyGive(s_pc_link_task);
}

// ---------------- 上传补传(重启不丢照片) ----------------
// uploaded.log 记录已成功上传的文件(SD 侧),开机与每次上传成功后扫描 SD,
// 将未上传的照片/扫描重新入队。SD 原件永不被删除,逻辑幂等。

static constexpr char PC_UPLOADED_LOG[] = "/uploaded.log";

static void pc_link_mark_uploaded(const char *relative_path)
{
    if (!relative_path || !relative_path[0] || !s_sd_mounted) return;
    if (xSemaphoreTake(s_sd_log_mutex, pdMS_TO_TICKS(300)) != pdTRUE) return;
    FILE *f = fopen((std::string(SD_MOUNT_POINT) + PC_UPLOADED_LOG).c_str(), "a");
    if (f) {
        fprintf(f, "%s\n", relative_path);
        fflush(f);
        fsync(fileno(f));
        fclose(f);
    }
    xSemaphoreGive(s_sd_log_mutex);
}

static void pc_link_enqueue_pending_from_sd()
{
    if (!s_sd_mounted || !s_pc_upload_mutex) return;
    std::set<std::string> uploaded;
    FILE *f = fopen((std::string(SD_MOUNT_POINT) + PC_UPLOADED_LOG).c_str(), "r");
    if (f) {
        char line[160];
        while (fgets(line, sizeof(line), f)) {
            std::string item = line;
            while (!item.empty() && (item.back() == '\n' || item.back() == '\r')) item.pop_back();
            if (!item.empty() && item[0] == '/') uploaded.insert(item);
        }
        fclose(f);
    }
    const auto scan_dir = [&](const char *dir_prefix, const char *suffix,
                              const char *name_prefix) {
        const std::string full = std::string(SD_MOUNT_POINT) + dir_prefix;
        DIR *dir = opendir(full.c_str());
        if (!dir) return;
        const size_t suffix_len = strlen(suffix);
        const size_t prefix_len = strlen(name_prefix);
        struct dirent *entry = nullptr;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name = entry->d_name;
            if (name == "." || name == "..") continue;
            if (name.size() < suffix_len ||
                name.compare(name.size() - suffix_len, suffix_len, suffix) != 0) continue;
            if (prefix_len > 0 && name.compare(0, prefix_len, name_prefix) != 0) continue;
            const std::string rel = std::string(dir_prefix) + "/" + name;
            if (uploaded.find(rel) == uploaded.end()) {
                // 补扫模式:队列满时跳过不丢队首,等待槽位腾出
                pc_link_queue_upload(rel.c_str(), false);
            }
        }
        closedir(dir);
    };
    // 根目录:照片(*.jpg)与扫描(scan_*.csv);dataset_capture:数据集照片
    scan_dir("", ".jpg", "");
    scan_dir("", ".csv", "scan_");
    scan_dir("/dataset_capture", ".jpg", "");
}

static void pc_link_task(void *)
{
    while (true) {
        if (!s_pc_link_enabled.load(std::memory_order_acquire)) {
            s_pc_pairing_active.store(false, std::memory_order_release);
            s_pc_link_connected.store(false, std::memory_order_release);
            wifi_stop_if_started();
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
            continue;
        }
        if (!pc_binding_snapshot().valid) {
            s_web_busy = true;
            const esp_err_t err = pc_start_pairing_ap();
            s_web_busy = false;
            if (err == ESP_OK) pc_pairing_loop();
            wifi_stop_if_started();
            if (!pc_binding_snapshot().valid) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        s_web_busy = true;
        esp_ip4_addr_t gateway = {};
        const bool station_ready = pc_connect_station(&gateway);
        s_web_busy = false;
        if (!station_ready) {
            dashboard_log_event("pc_link", "PC hotspot not found; retrying");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        int socket_fd = pc_open_receiver_socket(gateway);
        if (socket_fd < 0) {
            dashboard_log_event("pc_link", "PC receiver unavailable; retrying");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        s_pc_link_connected.store(true, std::memory_order_release);
        dashboard_log_event("pc_link", "PC receiver connected");
        ESP_LOGI("pc_link", "PC receiver connected stack_free=%u",
                 static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
        // 每次连上 PC 后补扫 SD:把拍照入队期间因任务阻塞而丢失的
        // 未上传照片/扫描重新入队(仅开机补扫不够,照片可能在重连窗口内拍摄)。
        pc_link_enqueue_pending_from_sd();
        int64_t last_telemetry_us = 0;
        int64_t last_frame_us = 0;
        bool link_ok = true;
        // 文件上传状态机:每轮发送一块(≤4KB)后回到主循环发送遥测,
        // 避免整文件同步发送长时间阻塞遥测导致 PC 判定“断连”。
        bool upload_active = false;
        char upload_path[PC_UPLOAD_PATH_LEN] = {};
        FILE *upload_file = nullptr;
        uint8_t upload_buffer[4096];
        uint32_t upload_crc = 0;
        uint64_t upload_remaining = 0;
        auto upload_finish = [&](bool success) {
            if (upload_file) { fclose(upload_file); upload_file = nullptr; }
            // 先复制路径再清空:done_path 不能指向 upload_path(清空后即失效)
            char done_path[PC_UPLOAD_PATH_LEN] = {};
            snprintf(done_path, sizeof(done_path), "%s", upload_path);
            upload_active = false;
            upload_path[0] = '\0';
            if (success && done_path[0]) {
                pc_pending_upload_pop();
                pc_link_mark_uploaded(done_path);
                pc_link_enqueue_pending_from_sd();
            } else if (!success && done_path[0]) {
                // 失败:弹出队首,避免死循环重试同一文件;补扫会重新入队
                pc_pending_upload_pop();
            }
        };
        while (link_ok && s_pc_link_enabled.load(std::memory_order_acquire) &&
               pc_binding_snapshot().valid) {
            const int64_t now_us = esp_timer_get_time();
            // 遥测优先:即使正在上传,也保持每 200ms 一次的遥测节奏
            if (now_us - last_telemetry_us >= 200000) {
                link_ok = pc_send_telemetry(socket_fd);
                last_telemetry_us = now_us;
            }
            bool room_active = false;
            dashboard_lock();
            room_active = s_room.active;
            dashboard_unlock();
            if (link_ok && room_active && now_us - last_frame_us >= 100000) {
                link_ok = pc_send_camera_frame(socket_fd);
                last_frame_us = now_us;
            }
            // 文件上传:增量发送,每轮最多一块
            if (link_ok) {
                if (!upload_active) {
                    char pending[PC_UPLOAD_PATH_LEN] = {};
                    if (pc_pending_upload_copy(pending, sizeof(pending))) {
                        const std::string full_path = std::string(SD_MOUNT_POINT) + pending;
                        struct stat file_stat = {};
                        if (stat(full_path.c_str(), &file_stat) != 0 ||
                            !S_ISREG(file_stat.st_mode) || file_stat.st_size < 0) {
                            pc_pending_upload_pop();
                            pc_link_enqueue_pending_from_sd();
                        } else {
                            FILE *file = fopen(full_path.c_str(), "rb");
                            if (!file) {
                                pc_pending_upload_pop();
                                pc_link_enqueue_pending_from_sd();
                            } else {
                                // 预计算 CRC
                                uint32_t crc = 0;
                                size_t count = 0;
                                while ((count = fread(upload_buffer, 1, sizeof(upload_buffer), file)) > 0) {
                                    crc = esp_rom_crc32_le(crc, upload_buffer, count);
                                }
                                if (ferror(file) || fseek(file, 0, SEEK_SET) != 0) {
                                    fclose(file);
                                    pc_pending_upload_pop();
                                    pc_link_enqueue_pending_from_sd();
                                } else {
                                    const char *base = strrchr(pending, '/');
                                    base = base && base[1] ? base + 1 : "scan.csv";
                                    char metadata[256];
                                    int metadata_length = snprintf(metadata, sizeof(metadata),
                                        "{\"name\":\"%s\",\"size\":%llu,\"crc32\":\"%08lx\"}", base,
                                        static_cast<unsigned long long>(file_stat.st_size),
                                        static_cast<unsigned long>(crc));
                                    const bool is_scan = strncmp(pending, "/scan_", 6) == 0;
                                    if (is_scan) {
                                        s_room_upload_state.store(RoomUploadState::UPLOADING, std::memory_order_release);
                                    }
                                    if (metadata_length <= 0 ||
                                        metadata_length >= static_cast<int>(sizeof(metadata)) ||
                                        !pc_send_packet(socket_fd, 4, metadata,
                                                        static_cast<size_t>(metadata_length))) {
                                        fclose(file);
                                        if (is_scan) {
                                            s_room_upload_state.store(RoomUploadState::RETRYING, std::memory_order_release);
                                        }
                                        pc_pending_upload_pop();
                                        pc_link_enqueue_pending_from_sd();
                                    } else {
                                        snprintf(upload_path, sizeof(upload_path), "%s", pending);
                                        upload_file = file;
                                        upload_active = true;
                                        upload_crc = crc;
                                        upload_remaining = static_cast<uint64_t>(file_stat.st_size);
                                    }
                                }
                            }
                        }
                    }
                }
                if (upload_active && upload_file) {
                    const size_t want = static_cast<size_t>(
                        upload_remaining > sizeof(upload_buffer) ? sizeof(upload_buffer) : upload_remaining);
                    const size_t got = fread(upload_buffer, 1, want, upload_file);
                    if (got > 0) {
                        link_ok = pc_send_packet(socket_fd, 5, upload_buffer, got);
                        upload_remaining -= got;
                    }
                    if (!link_ok || upload_remaining == 0 || ferror(upload_file)) {
                        if (link_ok) {
                            static constexpr char end[] = "{}";
                            link_ok = pc_send_packet(socket_fd, 6, end, sizeof(end) - 1);
                            char reply[256] = {};
                            if (link_ok) {
                                link_ok = pc_receive_reply(socket_fd, reply, sizeof(reply)) &&
                                          strstr(reply, "\"ok\":true") != nullptr;
                            }
                        }
                        const bool is_scan = strncmp(upload_path, "/scan_", 6) == 0;
                        // 记录日志用完整路径(upload_finish 会清空 upload_path)
                        char log_path[PC_UPLOAD_PATH_LEN] = {};
                        snprintf(log_path, sizeof(log_path), "%s", upload_path);
                        upload_finish(link_ok);
                        if (is_scan) {
                            s_room_upload_state.store(link_ok ? RoomUploadState::UPLOADED
                                                               : RoomUploadState::RETRYING,
                                                      std::memory_order_release);
                        }
                        if (link_ok && log_path[0]) {
                            dashboard_log_event("pc_link", std::string("uploaded ") + log_path);
                        }
                    }
                }
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        }
        if (upload_file) {
            fclose(upload_file);
            upload_file = nullptr;
        }
        s_pc_link_connected.store(false, std::memory_order_release);
        shutdown(socket_fd, SHUT_RDWR);
        close(socket_fd);
        dashboard_log_event("pc_link", "PC receiver disconnected");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void pc_link_start_once()
{
    if (!s_pc_upload_mutex) s_pc_upload_mutex = xSemaphoreCreateMutex();
    // 开机恢复 manual-only 标记:解除绑定后重启,配对循环仅接受手动绑定
    {
        nvs_handle_t flag_handle = 0;
        if (nvs_open("pc_link_flag", NVS_READONLY, &flag_handle) == ESP_OK) {
            uint8_t manual_only = 0;
            if (nvs_get_u8(flag_handle, "manual_only", &manual_only) == ESP_OK && manual_only) {
                s_pairing_manual_only.store(true, std::memory_order_release);
            }
            nvs_close(flag_handle);
        }
    }
    // 开机补传:把上次未上传的照片/扫描重新入队(重启不丢上传)
    pc_link_enqueue_pending_from_sd();
    if (!s_pc_binding_loaded.load(std::memory_order_acquire)) {
        PcBinding binding = {};
        pc_binding_load(&binding);
        dashboard_lock();
        s_pc_binding = binding;
        dashboard_unlock();
        s_pc_binding_loaded.store(true, std::memory_order_release);
    }
    const esp_err_t wifi_control_err = wifi_control_start_once();
    if (wifi_control_err != ESP_OK) {
        dashboard_set_error("Wi-Fi control task allocation failed: " +
                            std::string(esp_err_to_name(wifi_control_err)));
        return;
    }
    if (!s_pc_link_task) {
        // Networking, JPEG and SD paths need a large stack. NVS persistence is
        // delegated to internal-stack workers. Wi-Fi driver control is routed
        // through wifi_ctrl for the same flash-cache safety requirement.
        const BaseType_t created = xTaskCreateWithCaps(
            pc_link_task, "pc_link", 32768, nullptr, 5, &s_pc_link_task,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (created != pdPASS) {
            s_pc_link_task = nullptr;
            dashboard_set_error("PC link task allocation failed");
        }
    }
}

static esp_err_t wifi_init_once()
{
    ESP_RETURN_ON_ERROR(wifi_base_init_once(), "wifi", "base init");
    if (!s_wifi_sta_netif) {
        s_wifi_sta_netif = esp_netif_create_default_wifi_sta();
    }
    return wifi_configure_and_start(WIFI_MODE_STA, WIFI_IF_STA, nullptr);
}

static esp_err_t camera_http_init(bool fast_restore)
{
    if (s_camera_http_ready) {
        return ESP_OK;
    }
    if (!esp_psram_is_initialized()) {
        ESP_LOGE("camera", "PSRAM is required; refusing an internal-RAM frame buffer");
        return ESP_ERR_NO_MEM;
    }
    if (!s_camera_mutex) {
        s_camera_mutex = xSemaphoreCreateMutex();
        if (!s_camera_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    camera_config_t config = camera_jpeg_config(CAMERA_WEB_FRAME_SIZE,
                                                CAMERA_HTTP_JPEG_QUALITY, 2,
                                                CAMERA_GRAB_LATEST);
    // Keep DVP DMA running with two JPEG buffers and always decode the newest
    // completed frame.  Full-resolution still capture performs a complete
    // deinit/reinit cycle, so it no longer depends on changing the dimensions
    // of this live-preview DMA instance in place.
    ESP_RETURN_ON_ERROR(camera_use_shared_i2c(config), "camera", "camera shared I2C");

    esp_camera_deinit();
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        esp_camera_deinit();
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        auto apply_sensor_setting = [sensor](const char *name,
                                              int (*setter)(sensor_t *, int),
                                              int value) {
            const int result = setter ? setter(sensor, value) : -1;
            if (result != 0) {
                ESP_LOGW("camera", "OV5640 setting %s=%d rejected (%d)",
                         name, value, result);
            }
            return result;
        };
        if (sensor->set_framesize(sensor, CAMERA_LOCAL_FRAME_SIZE) != 0) {
            ESP_LOGW("camera", "OV5640 VGA frame-size setting rejected");
        }
        apply_sensor_setting("quality", sensor->set_quality, config.jpeg_quality);
        apply_sensor_setting("brightness", sensor->set_brightness, -1);
        apply_sensor_setting("contrast", sensor->set_contrast, 2);
        apply_sensor_setting("saturation", sensor->set_saturation, 0);
        apply_sensor_setting("sharpness", sensor->set_sharpness, 1);
        apply_sensor_setting("denoise", sensor->set_denoise, 2);
        apply_sensor_setting("whitebal", sensor->set_whitebal, 1);
        apply_sensor_setting("awb_gain", sensor->set_awb_gain, 1);
        apply_sensor_setting("exposure_ctrl", sensor->set_exposure_ctrl, 1);
        apply_sensor_setting("gain_ctrl", sensor->set_gain_ctrl, 1);
        // The wide-angle module tends to wash out light walls.  Lower the
        // OV5640 auto-exposure target while keeping AEC enabled so it can
        // still adapt between rooms.
        apply_sensor_setting("ae_level", sensor->set_ae_level, -2);
        apply_sensor_setting("lenc", sensor->set_lenc, 1);
        apply_sensor_setting("raw_gma", sensor->set_raw_gma, 1);
        apply_sensor_setting("bpc", sensor->set_bpc, 1);
        apply_sensor_setting("wpc", sensor->set_wpc, 1);
#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
        if (!fast_restore && esp_camera_af_is_supported(sensor)) {
            esp_camera_af_config_t af_cfg = {};
            af_cfg.mode = ESP_CAMERA_AF_MODE_AUTO;
            af_cfg.timeout_ms = CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS;
            if (esp_camera_af_init(sensor, &af_cfg) == ESP_OK) {
                esp_camera_af_trigger(sensor);
                esp_camera_af_status_t af_status = {};
                esp_camera_af_wait(sensor, CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS, &af_status);
            }
        }
#endif
        // Do not expose the first unstable auto-exposure/white-balance frames
        // to the LCD or web clients.
        const int warmup_frames = fast_restore ? 2 : 8;
        for (int i = 0; i < warmup_frames; ++i) {
            camera_fb_t *warmup = esp_camera_fb_get();
            if (warmup) esp_camera_fb_return(warmup);
        }
        ESP_LOGI("camera", "OV5640 tuned: VGA JPEG Q%d, brightness=-1 contrast=2 ae=-2 sharpness=1 denoise=2 warmup=%d%s",
                 config.jpeg_quality, warmup_frames, fast_restore ? " async_af" : "");
    }
    if (!s_preview_decoder) {
        log_memory("before SIMD JPEG decoder init");
        camera_jpeg_decoder_config_t decoder_config = CAMERA_JPEG_DECODER_DEFAULT_CONFIG();
        err = camera_jpeg_decoder_create(&decoder_config, &s_preview_decoder);
        if (err != ESP_OK) {
            esp_camera_deinit();
            ESP_LOGE("camera", "SIMD JPEG decoder init failed: %s", esp_err_to_name(err));
            return err;
        }
        log_memory("after SIMD JPEG decoder init");
    }
    s_camera_http_ready = true;
    return ESP_OK;
}

static void json_append_escaped(std::string &out, const char *s)
{
    out += "\"";
    if (s) {
        for (; *s; ++s) {
            switch (*s) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(*s) < 0x20) {
                    char b[8];
                    snprintf(b, sizeof(b), "\\u%04x", static_cast<unsigned char>(*s));
                    out += b;
                } else {
                    out += *s;
                }
            }
        }
    }
    out += "\"";
}

static esp_err_t http_send_json(httpd_req_t *req, const std::string &json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json.c_str(), json.size());
}

static std::string dashboard_state_json()
{
    DashboardState snap;
    SingleDistanceSnapshot single;
    dashboard_lock();
    snap = s_dash;
    single = s_single_distance.snapshot();
    dashboard_unlock();

    int raw_sda = gpio_get_level(PIN_I2C_SDA);
    int raw_scl = gpio_get_level(PIN_I2C_SCL);

    char b[1792];
    snprintf(b, sizeof(b),
             "{\"uptime_ms\":%lld,\"runtime\":%s,\"sd\":%s,\"camera\":%s,\"i2c\":{\"sda\":%d,\"scl\":%d},"
             "\"bno\":{\"ok\":%s,\"count\":%lu,\"status\":%u,"
             "\"accel\":[%.4f,%.4f,%.4f],\"gyro\":[%.5f,%.5f,%.5f],"
             "\"quat\":[%.5f,%.5f,%.5f,%.5f],\"mag\":[%.3f,%.3f,%.3f]},"
             "\"fusion\":{\"position_m\":[%.5f,%.5f,%.5f],\"velocity_mps\":[%.5f,%.5f],"
             "\"ypr_rad\":[%.5f,%.5f,%.5f],\"confidence\":%.3f,\"stationary\":%s,"
             "\"session_id\":%lu,\"point_count\":%u,\"point1_m\":[%.5f,%.5f,%.5f],\"point2_m\":[%.5f,%.5f,%.5f],"
             "\"point_distance_m\":%.5f},"
             "\"room\":{\"active\":%s,\"complete\":%s,\"session_id\":%lu,\"point_count\":%u,"
             "\"last_segment_m\":%.5f,\"open_perimeter_m\":%.5f,\"closure_m\":%.5f,\"area_xy_m2\":%.5f},"
             "\"laser\":{\"ok\":%s,\"busy\":%s,\"count\":%lu,\"distance_mm\":%ld,\"raw\":",
             static_cast<long long>(esp_timer_get_time() / 1000),
             snap.runtime_started ? "true" : "false",
             snap.sd_ready ? "true" : "false",
             s_camera_http_ready ? "true" : "false",
             raw_sda, raw_scl,
             snap.bno_ready ? "true" : "false",
             static_cast<unsigned long>(snap.bno_count),
             snap.bno_status,
             static_cast<double>(snap.ax), static_cast<double>(snap.ay), static_cast<double>(snap.az),
             static_cast<double>(snap.gx), static_cast<double>(snap.gy), static_cast<double>(snap.gz),
             static_cast<double>(snap.qi), static_cast<double>(snap.qj), static_cast<double>(snap.qk), static_cast<double>(snap.qr),
             static_cast<double>(snap.mx), static_cast<double>(snap.my), static_cast<double>(snap.mz),
             static_cast<double>(snap.path_x), static_cast<double>(snap.path_y), static_cast<double>(snap.path_z),
             static_cast<double>(snap.fusion_vx), static_cast<double>(snap.fusion_vy),
             static_cast<double>(snap.fusion_yaw), static_cast<double>(snap.fusion_pitch), static_cast<double>(snap.fusion_roll),
             static_cast<double>(snap.fusion_confidence), snap.fusion_stationary ? "true" : "false",
             static_cast<unsigned long>(snap.measure_session_id), snap.measure_point_count,
             static_cast<double>(snap.point1_x), static_cast<double>(snap.point1_y), static_cast<double>(snap.point1_z),
             static_cast<double>(snap.point2_x), static_cast<double>(snap.point2_y), static_cast<double>(snap.point2_z),
             static_cast<double>(snap.point_distance_m),
             snap.room_active ? "true" : "false", snap.room_complete ? "true" : "false",
             static_cast<unsigned long>(snap.room_session_id), snap.room_point_count,
             static_cast<double>(snap.room_last_segment_m), static_cast<double>(snap.room_open_perimeter_m),
             static_cast<double>(snap.room_closure_m), static_cast<double>(snap.room_area_xy_m2),
             snap.laser_ready ? "true" : "false", snap.laser_busy ? "true" : "false",
             static_cast<unsigned long>(snap.laser_count), static_cast<long>(snap.laser_mm));
    std::string out = b;
    json_append_escaped(out, snap.laser_raw);
    char c[768];
    snprintf(c, sizeof(c),
             "},\"single\":{\"active\":%s,\"state\":%u,\"error\":%ld,\"reference\":%u,"
             "\"result_valid\":%s,\"distance_mm\":%ld,\"temporary_count\":%u,\"saved_record_id\":%lu},"
             "\"keys\":{\"measure\":%s,\"back\":%s,\"ok\":%s,\"raw\":%u,\"measure_count\":%lu,\"ok_count\":%lu},"
             "\"photo\":{\"count\":%lu,\"last\":",
             single.page_active ? "true" : "false",
             static_cast<unsigned>(single.state),
             static_cast<long>(single.error),
             static_cast<unsigned>(single.selected_reference),
             single.result_valid ? "true" : "false",
             static_cast<long>(single.result.distance_mm),
             single.history_count,
             static_cast<unsigned long>(single.result.saved_record_id),
             snap.key_measure ? "true" : "false",
             snap.key_back ? "true" : "false",
             snap.key_ok ? "true" : "false",
             snap.key_raw,
             static_cast<unsigned long>(snap.measure_press_count),
             static_cast<unsigned long>(snap.ok_press_count),
             static_cast<unsigned long>(snap.photo_count));
    out += c;
    json_append_escaped(out, snap.last_photo);
    out += "},\"error\":";
    json_append_escaped(out, snap.last_error);
    out += "}";
    return out;
}

static bool get_query_value(httpd_req_t *req, const char *key, char *value, size_t value_len)
{
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1 || qlen > 256) {
        return false;
    }
    char query[256] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    if (httpd_query_key_value(query, key, value, value_len) != ESP_OK) {
        return false;
    }

    // esp_http_server extracts query values but intentionally leaves percent
    // encoding untouched.  Decode in place before validating SD paths so a
    // browser's encodeURIComponent("/") is treated as '/' rather than a file
    // literally named "%2F".  Reject malformed escapes and embedded NULs.
    auto hex_value = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    char *read = value;
    char *write = value;
    while (*read) {
        if (*read == '%') {
            const int hi = hex_value(read[1]);
            const int lo = read[1] ? hex_value(read[2]) : -1;
            if (hi < 0 || lo < 0) {
                return false;
            }
            const char decoded = static_cast<char>((hi << 4) | lo);
            if (decoded == '\0') {
                return false;
            }
            *write++ = decoded;
            read += 3;
        } else {
            *write++ = (*read == '+') ? ' ' : *read;
            ++read;
        }
    }
    *write = '\0';
    return true;
}

static bool sanitize_sd_rel_path(const char *in, std::string &rel)
{
    rel = in && in[0] ? in : "/";
    if (rel[0] != '/') {
        rel = "/" + rel;
    }
    if (rel.find("..") != std::string::npos || rel.find('\\') != std::string::npos ||
        rel.find("//") != std::string::npos || rel.find('\0') != std::string::npos) {
        return false;
    }
    while (rel.size() > 1 && rel.back() == '/') rel.pop_back();
    return true;
}

static esp_err_t http_json_error(httpd_req_t *req, const char *status, const char *message)
{
    httpd_resp_set_status(req, status);
    std::string out = "{\"ok\":false,\"error\":";
    json_append_escaped(out, message);
    out += "}";
    return http_send_json(req, out);
}

static esp_err_t api_state_handler(httpd_req_t *req)
{
    return http_send_json(req, dashboard_state_json());
}

static esp_err_t api_time_handler(httpd_req_t *req)
{
    char value[24] = {};
    if (!get_query_value(req, "epoch_ms", value, sizeof(value))) {
        return http_json_error(req, "400 Bad Request", "missing epoch_ms");
    }
    char *end = nullptr;
    const int64_t epoch_ms = strtoll(value, &end, 10);
    if (!end || *end != '\0' || epoch_ms < 1609459200000LL ||
        epoch_ms > 4102444800000LL) {
        return http_json_error(req, "400 Bad Request", "invalid epoch_ms");
    }
    struct timeval timestamp = {};
    timestamp.tv_sec = static_cast<time_t>(epoch_ms / 1000);
    timestamp.tv_usec = static_cast<suseconds_t>((epoch_ms % 1000) * 1000);
    if (settimeofday(&timestamp, nullptr) != 0) {
        return http_json_error(req, "500 Internal Server Error", "settimeofday failed");
    }
    setenv("TZ", "CST-8", 1);
    tzset();
    dashboard_log_event("time", "browser clock synchronized");
    return http_send_json(req, "{\"ok\":true,\"time_synced\":true}");
}

static esp_err_t api_measure_handler(httpd_req_t *req)
{
    dashboard_lock();
    const bool single_page_active = s_single_distance.snapshot().page_active;
    dashboard_unlock();
    if (single_page_active) {
        device_ui_single_trigger();
        dashboard_log_event("api", "single-distance trigger requested");
        return http_send_json(req, "{\"ok\":true,\"action\":\"single_trigger\"}");
    }
    if (!s_laser_active_requested.load(std::memory_order_acquire)) {
        return http_json_error(req, "409 Conflict", "enter a measurement page before using the laser");
    }
    s_laser_measure_request.store(true, std::memory_order_release);
    if (s_action_task) xTaskNotifyGive(s_action_task);
    dashboard_log_event("api", "measure requested");
    return http_send_json(req, "{\"ok\":true,\"action\":\"measure\"}");
}

static esp_err_t api_capture_handler(httpd_req_t *req)
{
    s_photo_request.store(true, std::memory_order_release);
    dashboard_log_event("api", "capture requested");
    return http_send_json(req, "{\"ok\":true,\"action\":\"capture\"}");
}

static std::string i2c_diag_json(const I2cLineDiag &d)
{
    std::string out = "{\"ok\":";
    out += (d.idle_sda == 1 && d.idle_scl == 1) ? "true" : "false";
    char b[320];
    snprintf(b, sizeof(b),
             ",\"idle\":[%d,%d],\"scl_low\":[%d,%d],\"scl_release\":[%d,%d],"
             "\"sda_low\":[%d,%d],\"sda_release\":[%d,%d],\"cam_rst_low\":[%d,%d],"
             "\"cam_rst_high\":[%d,%d],\"camera_reset_sampled\":%s,\"verdict\":",
             d.idle_sda, d.idle_scl,
             d.scl_low_sda, d.scl_low_scl,
             d.scl_release_sda, d.scl_release_scl,
             d.sda_low_sda, d.sda_low_scl,
             d.sda_release_sda, d.sda_release_scl,
             d.cam_rst_low_sda, d.cam_rst_low_scl,
             d.cam_rst_high_sda, d.cam_rst_high_scl,
             d.camera_reset_sampled ? "true" : "false");
    out += b;
    json_append_escaped(out, i2c_line_verdict(d).c_str());
    out += ",\"detail\":";
    json_append_escaped(out, i2c_line_detail(d).c_str());
    out += "}";
    return out;
}

static esp_err_t api_i2c_diag_handler(httpd_req_t *req)
{
    return http_send_json(req, i2c_diag_json(i2c_runtime_line_diag()));
}

static esp_err_t api_i2c_recover_handler(httpd_req_t *req)
{
    if (s_camera_http_ready) {
        httpd_resp_set_status(req, "409 Conflict");
        return http_send_json(req,
            "{\"ok\":false,\"error\":\"camera is active on shared I2C; reboot for safe bus recovery\"}");
    }
    i2c_release_bus();
    SCCB_Deinit();
    i2c_gpio_bus_recover("api_i2c_recover");
    dashboard_log_event("api", "i2c recover requested");
    return http_send_json(req, i2c_diag_json(i2c_line_diag(false)));
}

static esp_err_t api_files_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return http_send_json(req, "{\"ok\":false,\"error\":\"SD not mounted\"}");
    }
    char qpath[128] = "/";
    get_query_value(req, "path", qpath, sizeof(qpath));
    std::string rel;
    if (!sanitize_sd_rel_path(qpath, rel)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return http_send_json(req, "{\"ok\":false,\"error\":\"bad path\"}");
    }
    std::string full = std::string(SD_MOUNT_POINT) + rel;
    DIR *dir = opendir(full.c_str());
    if (!dir) {
        httpd_resp_set_status(req, "404 Not Found");
        return http_send_json(req, "{\"ok\":false,\"error\":\"directory not found\"}");
    }
    std::string out = "{\"ok\":true,\"path\":";
    json_append_escaped(out, rel.c_str());
    out += ",\"items\":[";
    bool first = true;
    while (dirent *e = readdir(dir)) {
        if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) {
            continue;
        }
        std::string child_rel = rel == "/" ? ("/" + std::string(e->d_name)) : (rel + "/" + e->d_name);
        std::string child_full = std::string(SD_MOUNT_POINT) + child_rel;
        struct stat st = {};
        bool stat_ok = stat(child_full.c_str(), &st) == 0;
        if (!first) {
            out += ",";
        }
        first = false;
        out += "{\"name\":";
        json_append_escaped(out, e->d_name);
        out += ",\"path\":";
        json_append_escaped(out, child_rel.c_str());
        out += ",\"dir\":";
        out += (stat_ok && S_ISDIR(st.st_mode)) ? "true" : "false";
        out += ",\"size\":";
        out += stat_ok ? std::to_string(static_cast<long long>(st.st_size)) : "0";
        out += "}";
    }
    closedir(dir);
    out += "]}";
    return http_send_json(req, out);
}

static esp_err_t http_raw_send_all(httpd_req_t *req, const char *data, size_t length)
{
    size_t sent_total = 0;
    while (sent_total < length) {
        const int sent = httpd_send(req, data + sent_total, length - sent_total);
        if (sent <= 0) return ESP_ERR_HTTPD_RESP_SEND;
        sent_total += static_cast<size_t>(sent);
    }
    return ESP_OK;
}

static const char *sd_file_content_type(const std::string &path)
{
    const size_t dot = path.rfind('.');
    std::string ext = dot == std::string::npos ? std::string() : path.substr(dot);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
    if (ext == ".png") return "image/png";
    if (ext == ".csv") return "text/csv; charset=utf-8";
    if (ext == ".txt" || ext == ".log" || ext == ".md") return "text/plain; charset=utf-8";
    if (ext == ".json") return "application/json";
    if (ext == ".pdf") return "application/pdf";
    if (ext == ".dxf") return "application/dxf";
    return "application/octet-stream";
}

static std::string rfc5987_filename(const char *name)
{
    static constexpr char hex[] = "0123456789ABCDEF";
    std::string encoded;
    if (!name) return "download";
    encoded.reserve(strlen(name) * 3);
    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(name); *p; ++p) {
        const unsigned char c = *p;
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            encoded.push_back(static_cast<char>(c));
        } else {
            encoded.push_back('%');
            encoded.push_back(hex[c >> 4]);
            encoded.push_back(hex[c & 0x0F]);
        }
    }
    return encoded.empty() ? "download" : encoded;
}

static esp_err_t api_file_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "SD not mounted");
    }
    char qpath[160] = {};
    if (!get_query_value(req, "path", qpath, sizeof(qpath))) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "missing path");
    }
    std::string rel;
    if (!sanitize_sd_rel_path(qpath, rel)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "bad path");
    }
    std::string full = std::string(SD_MOUNT_POINT) + rel;
    struct stat file_st = {};
    if (rel == "/" || stat(full.c_str(), &file_st) != 0 || !S_ISREG(file_st.st_mode)) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "not found");
    }
    FILE *f = fopen(full.c_str(), "rb");
    if (!f) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "not found");
    }
    const char *base = strrchr(rel.c_str(), '/');
    base = base && *(base + 1) ? base + 1 : "download";
    char ascii_name[128] = {};
    size_t ascii_pos = 0;
    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(base);
         *p && ascii_pos + 1 < sizeof(ascii_name); ++p) {
        const unsigned char c = *p;
        ascii_name[ascii_pos++] = (c < 0x80 && (std::isalnum(c) || c == '.' || c == '-' || c == '_'))
                                      ? static_cast<char>(c)
                                      : '_';
    }
    if (ascii_pos == 0) snprintf(ascii_name, sizeof(ascii_name), "download");
    const std::string encoded_name = rfc5987_filename(base);

    const uint64_t file_size = static_cast<uint64_t>(file_st.st_size);
    uint64_t range_start = 0;
    uint64_t range_end = file_size > 0 ? file_size - 1 : 0;
    bool partial = false;
    char range_value[80] = {};
    const size_t range_len = httpd_req_get_hdr_value_len(req, "Range");
    if (range_len > 0 && range_len < sizeof(range_value) &&
        httpd_req_get_hdr_value_str(req, "Range", range_value, sizeof(range_value)) == ESP_OK &&
        strchr(range_value, ',') == nullptr) {
        unsigned long long requested_start = 0;
        unsigned long long requested_end = 0;
        const int fields = sscanf(range_value, "bytes=%llu-%llu", &requested_start, &requested_end);
        if (fields >= 1) {
            if (requested_start >= file_size) {
                fclose(f);
                char unsatisfied[64];
                snprintf(unsatisfied, sizeof(unsatisfied), "bytes */%llu",
                         static_cast<unsigned long long>(file_size));
                httpd_resp_set_status(req, "416 Range Not Satisfiable");
                httpd_resp_set_hdr(req, "Content-Range", unsatisfied);
                return httpd_resp_send(req, nullptr, 0);
            }
            range_start = requested_start;
            range_end = fields >= 2 ? std::min<uint64_t>(requested_end, file_size - 1)
                                    : file_size - 1;
            if (range_end >= range_start) {
                partial = true;
            } else {
                range_start = 0;
                range_end = file_size - 1;
            }
        }
    }

    if (range_start > 0 && fseek(f, static_cast<long>(range_start), SEEK_SET) != 0) {
        fclose(f);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "seek failed");
    }

    const uint64_t content_length = file_size == 0 ? 0 : range_end - range_start + 1;
    char content_range[96] = {};
    if (partial) {
        snprintf(content_range, sizeof(content_range),
                 "Content-Range: bytes %llu-%llu/%llu\r\n",
                 static_cast<unsigned long long>(range_start),
                 static_cast<unsigned long long>(range_end),
                 static_cast<unsigned long long>(file_size));
    }
    char header[1024];
    const int header_len = snprintf(
        header, sizeof(header),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %llu\r\n"
        "Content-Disposition: attachment; filename=\"%s\"; filename*=UTF-8''%s\r\n"
        "Accept-Ranges: bytes\r\n"
        "Cache-Control: no-store\r\n"
        "X-Content-Type-Options: nosniff\r\n"
        "%s"
        "Connection: close\r\n\r\n",
        partial ? "206 Partial Content" : "200 OK",
        sd_file_content_type(rel),
        static_cast<unsigned long long>(content_length),
        ascii_name, encoded_name.c_str(), content_range);
    if (header_len <= 0 || header_len >= static_cast<int>(sizeof(header))) {
        fclose(f);
        return ESP_ERR_HTTPD_RESP_HDR;
    }

    // Do not place the transfer buffer on the HTTP task stack.  Besides the
    // response header and C++ path strings, the TCP/HTTP call chain needs
    // several KiB of stack; the former 4 KiB local array could overflow the
    // 8 KiB server task exactly when a download began.  A fixed PSRAM buffer
    // also keeps the transfer from consuming scarce Wi-Fi internal RAM.
    constexpr size_t transfer_buffer_size = 4096;
    char *buf = static_cast<char *>(
        heap_caps_malloc(transfer_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buf) {
        fclose(f);
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "download buffer unavailable");
    }

    ESP_LOGI("sd_http", "download begin %s size=%llu stack_free=%u internal_free=%u largest=%u",
             rel.c_str(), static_cast<unsigned long long>(content_length),
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));

    esp_err_t err = http_raw_send_all(req, header, static_cast<size_t>(header_len));
    uint64_t remaining = content_length;
    while (err == ESP_OK && remaining > 0) {
        const size_t wanted = static_cast<size_t>(std::min<uint64_t>(remaining, transfer_buffer_size));
        const size_t count = fread(buf, 1, wanted, f);
        if (count == 0) {
            err = ESP_FAIL;
            break;
        }
        err = http_raw_send_all(req, buf, count);
        remaining -= count;
    }
    fclose(f);
    heap_caps_free(buf);
    ESP_LOGI("sd_http", "download end %s offset=%llu bytes=%llu result=%s stack_free=%u",
             rel.c_str(), static_cast<unsigned long long>(range_start),
             static_cast<unsigned long long>(content_length), esp_err_to_name(err),
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    return err;
}

static esp_err_t api_fs_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) return http_json_error(req, "503 Service Unavailable", "SD not mounted");
    char action[16] = {}, qpath[160] = {}, qdest[160] = {};
    if (!get_query_value(req, "action", action, sizeof(action)) ||
        !get_query_value(req, "path", qpath, sizeof(qpath))) {
        return http_json_error(req, "400 Bad Request", "missing action or path");
    }
    std::string rel;
    if (!sanitize_sd_rel_path(qpath, rel) || rel == "/") {
        return http_json_error(req, "400 Bad Request", "invalid or protected path");
    }
    std::string full = std::string(SD_MOUNT_POINT) + rel;

    if (strcmp(action, "delete") == 0) {
        struct stat st = {};
        if (stat(full.c_str(), &st) != 0) return http_json_error(req, "404 Not Found", "not found");
        int rc = S_ISDIR(st.st_mode) ? rmdir(full.c_str()) : remove(full.c_str());
        if (rc != 0) return http_json_error(req, "409 Conflict", "delete failed; directory must be empty");
    } else if (strcmp(action, "mkdir") == 0) {
        if (mkdir(full.c_str(), 0775) != 0) return http_json_error(req, "409 Conflict", "create directory failed");
    } else if (strcmp(action, "rename") == 0) {
        if (!get_query_value(req, "dest", qdest, sizeof(qdest))) return http_json_error(req, "400 Bad Request", "missing destination");
        std::string dest_rel;
        if (!sanitize_sd_rel_path(qdest, dest_rel) || dest_rel == "/") return http_json_error(req, "400 Bad Request", "bad destination");
        std::string dest = std::string(SD_MOUNT_POINT) + dest_rel;
        struct stat st = {};
        if (stat(dest.c_str(), &st) == 0) return http_json_error(req, "409 Conflict", "destination exists");
        if (rename(full.c_str(), dest.c_str()) != 0) return http_json_error(req, "409 Conflict", "rename failed");
    } else if (strcmp(action, "save") == 0) {
        if (static_cast<size_t>(req->content_len) > WEB_TEXT_FILE_MAX_BYTES) {
            return http_json_error(req, "413 Payload Too Large", "text file limit is 64 KiB");
        }
        std::string tmp = full + ".webtmp";
        FILE *f = fopen(tmp.c_str(), "wb");
        if (!f) return http_json_error(req, "409 Conflict", "cannot open temporary file");
        char buf[1024];
        int remaining = req->content_len;
        bool ok = true;
        while (remaining > 0) {
            int n = httpd_req_recv(req, buf, std::min<int>(remaining, sizeof(buf)));
            if (n <= 0 || fwrite(buf, 1, n, f) != static_cast<size_t>(n)) { ok = false; break; }
            remaining -= n;
        }
        if (fclose(f) != 0) ok = false;
        if (!ok) { remove(tmp.c_str()); return http_json_error(req, "500 Internal Server Error", "write failed"); }
        std::string backup = full + ".webbak";
        remove(backup.c_str());
        const bool had_original = rename(full.c_str(), backup.c_str()) == 0;
        if (rename(tmp.c_str(), full.c_str()) != 0) {
            if (had_original) rename(backup.c_str(), full.c_str());
            remove(tmp.c_str());
            return http_json_error(req, "500 Internal Server Error", "commit failed; original restored");
        }
        if (had_original) remove(backup.c_str());
    } else {
        return http_json_error(req, "400 Bad Request", "unknown action");
    }
    dashboard_log_event("web_fs", (std::string(action) + " " + rel).c_str());
    return http_send_json(req, "{\"ok\":true}");
}

static esp_err_t dashboard_page_handler(httpd_req_t *req)
{
    const size_t body_len = strlen(web_ui::kDashboardHtml);
    char header[256];
    const int header_len = snprintf(header, sizeof(header),
                                    "HTTP/1.1 200 OK\r\n"
                                    "Content-Type: text/html; charset=utf-8\r\n"
                                    "Content-Length: %u\r\n"
                                    "Cache-Control: no-store\r\n"
                                    "Connection: close\r\n\r\n",
                                    static_cast<unsigned>(body_len));
    if (header_len <= 0 || header_len >= static_cast<int>(sizeof(header))) {
        return ESP_ERR_HTTPD_RESP_HDR;
    }
    ESP_RETURN_ON_ERROR(http_raw_send_all(req, header, static_cast<size_t>(header_len)),
                        "cam_http", "dashboard header");
    for (size_t offset = 0; offset < body_len; offset += 4096) {
        const size_t count = std::min<size_t>(4096, body_len - offset);
        ESP_RETURN_ON_ERROR(http_raw_send_all(req, web_ui::kDashboardHtml + offset, count),
                            "cam_http", "dashboard body");
    }
    return ESP_OK;
}

static esp_err_t file_manager_page_handler(httpd_req_t *req)
{
    const size_t body_len = strlen(web_ui::kFileManagerHtml);
    char header[256];
    const int header_len = snprintf(header, sizeof(header),
                                    "HTTP/1.1 200 OK\r\n"
                                    "Content-Type: text/html; charset=utf-8\r\n"
                                    "Content-Length: %u\r\n"
                                    "Cache-Control: no-store\r\n"
                                    "Connection: close\r\n\r\n",
                                    static_cast<unsigned>(body_len));
    if (header_len <= 0 || header_len >= static_cast<int>(sizeof(header))) {
        return ESP_ERR_HTTPD_RESP_HDR;
    }
    ESP_RETURN_ON_ERROR(http_raw_send_all(req, header, static_cast<size_t>(header_len)),
                        "cam_http", "file page header");
    for (size_t offset = 0; offset < body_len; offset += 4096) {
        const size_t count = std::min<size_t>(4096, body_len - offset);
        ESP_RETURN_ON_ERROR(http_raw_send_all(req, web_ui::kFileManagerHtml + offset, count),
                            "cam_http", "file page body");
    }
    return ESP_OK;
}

static esp_err_t camera_jpg_handler(httpd_req_t *req)
{
    if (!s_camera_http_ready || !s_camera_mutex) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not ready");
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera busy");
        return ESP_FAIL;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        xSemaphoreGive(s_camera_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "capture failed");
        return ESP_FAIL;
    }

    uint8_t *jpg = fb->buf;
    size_t jpg_len = fb->len;
    bool converted = false;
    if (fb->format != PIXFORMAT_JPEG) {
        jpg = nullptr;
        jpg_len = 0;
        converted = frame2jpg(fb, 80, &jpg, &jpg_len);
        if (!converted || !jpg || jpg_len == 0) {
            esp_camera_fb_return(fb);
            xSemaphoreGive(s_camera_mutex);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "jpeg convert failed");
            return ESP_FAIL;
        }
    }

    // Copy the frame into PSRAM before doing any socket I/O.  A slow or
    // disconnected browser must never keep the camera frame and mutex held,
    // otherwise both the LCD preview and every later camera request stall.
    uint8_t *send_copy = static_cast<uint8_t *>(
        heap_caps_malloc(jpg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!send_copy) {
        if (converted) free(jpg);
        esp_camera_fb_return(fb);
        xSemaphoreGive(s_camera_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "jpeg buffer unavailable");
        return ESP_ERR_NO_MEM;
    }
    memcpy(send_copy, jpg, jpg_len);
    if (converted) free(jpg);
    esp_camera_fb_return(fb);
    xSemaphoreGive(s_camera_mutex);

    char download_value[8] = {};
    const bool download = get_query_value(req, "download", download_value,
                                          sizeof(download_value)) &&
                          strcmp(download_value, "0") != 0;
    char header[384];
    const int header_len = snprintf(
        header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %u\r\n"
        "Cache-Control: no-store\r\n"
        "X-Image-Source: OV5640 VGA original JPEG\r\n"
        "%s"
        "Connection: close\r\n\r\n",
        static_cast<unsigned>(jpg_len),
        download ? "Content-Disposition: attachment; filename=\"ov5640_vga_q5.jpg\"\r\n" : "");
    esp_err_t err = ESP_ERR_HTTPD_RESP_HDR;
    if (header_len > 0 && header_len < static_cast<int>(sizeof(header))) {
        err = http_raw_send_all(req, header, static_cast<size_t>(header_len));
        for (size_t offset = 0; err == ESP_OK && offset < jpg_len; offset += 4096) {
            const size_t count = std::min<size_t>(4096, jpg_len - offset);
            err = http_raw_send_all(req, reinterpret_cast<const char *>(send_copy + offset), count);
        }
    }
    heap_caps_free(send_copy);
    if (err != ESP_OK) ESP_LOGW("cam_http", "single JPEG client disconnected: %s", esp_err_to_name(err));
    return err;
}

static esp_err_t camera_stream_handler(httpd_req_t *req)
{
    if (!s_camera_http_ready || !s_camera_mutex) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not ready");
        return ESP_FAIL;
    }

    static const char boundary[] = "--frame\r\n";
    char part[96];
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (!s_stream_stop_requested) {
        if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
            return ESP_FAIL;
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            xSemaphoreGive(s_camera_mutex);
            return ESP_FAIL;
        }

        uint8_t *jpg = fb->buf;
        size_t jpg_len = fb->len;
        bool converted = false;
        if (fb->format != PIXFORMAT_JPEG) {
            jpg = nullptr;
            jpg_len = 0;
            converted = frame2jpg(fb, 75, &jpg, &jpg_len);
        }

        esp_err_t err = ESP_OK;
        if (jpg && jpg_len > 0) {
            int hlen = snprintf(part, sizeof(part), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", static_cast<unsigned>(jpg_len));
            err = httpd_resp_send_chunk(req, boundary, strlen(boundary));
            if (err == ESP_OK) err = httpd_resp_send_chunk(req, part, hlen);
            if (err == ESP_OK) err = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(jpg), jpg_len);
            if (err == ESP_OK) err = httpd_resp_send_chunk(req, "\r\n", 2);
        } else {
            err = ESP_FAIL;
        }

        if (converted && jpg) {
            free(jpg);
        }
        esp_camera_fb_return(fb);
        xSemaphoreGive(s_camera_mutex);

        if (err != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(65));
    }
    return ESP_OK;
}

static esp_err_t camera_stream_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1:81/stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t camera_http_start()
{
    if (s_camera_httpd && s_stream_httpd) {
        return ESP_OK;
    }
    // Camera and LCD DMA resources are already allocated at this point.  Keep
    // subsequent medium/large HTTP bookkeeping and response allocations in
    // PSRAM so Wi-Fi retains the 64 KiB internal/DMA safety reserve.
    heap_caps_malloc_extmem_enable(1024);
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    // CONFIG_LWIP_MAX_SOCKETS is 10.  The main and stream servers consume
    // four sockets for their listen/control endpoints before any browser is
    // connected, so keep the combined client allowance at five.  The former
    // 7 + 2 configuration exhausted the global socket table as soon as a
    // browser opened video plus API polling (accept errno 23).
    config.max_open_sockets = 4;
    config.max_uri_handlers = 13;
    // This stack lives in PSRAM, so reserve enough space for file-system and
    // socket call chains without reducing the internal-RAM Wi-Fi headroom.
    config.stack_size = 12288;
    config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    ESP_RETURN_ON_ERROR(httpd_start(&s_camera_httpd, &config), "cam_http", "http start");

    httpd_uri_t root_uri = {};
    root_uri.uri = "/";
    root_uri.method = HTTP_GET;
    root_uri.handler = dashboard_page_handler;
    httpd_register_uri_handler(s_camera_httpd, &root_uri);

    httpd_uri_t file_manager_uri = {};
    file_manager_uri.uri = "/files";
    file_manager_uri.method = HTTP_GET;
    file_manager_uri.handler = file_manager_page_handler;
    httpd_register_uri_handler(s_camera_httpd, &file_manager_uri);

    httpd_uri_t jpg_uri = {};
    jpg_uri.uri = "/jpg";
    jpg_uri.method = HTTP_GET;
    jpg_uri.handler = camera_jpg_handler;
    httpd_register_uri_handler(s_camera_httpd, &jpg_uri);

    httpd_uri_t stream_uri = {};
    stream_uri.uri = "/stream";
    stream_uri.method = HTTP_GET;
    stream_uri.handler = camera_stream_redirect_handler;
    httpd_register_uri_handler(s_camera_httpd, &stream_uri);

    httpd_uri_t state_uri = {};
    state_uri.uri = "/api/state";
    state_uri.method = HTTP_GET;
    state_uri.handler = api_state_handler;
    httpd_register_uri_handler(s_camera_httpd, &state_uri);

    httpd_uri_t time_uri = {};
    time_uri.uri = "/api/time";
    time_uri.method = HTTP_GET;
    time_uri.handler = api_time_handler;
    httpd_register_uri_handler(s_camera_httpd, &time_uri);

    httpd_uri_t i2c_diag_uri = {};
    i2c_diag_uri.uri = "/api/i2c_diag";
    i2c_diag_uri.method = HTTP_GET;
    i2c_diag_uri.handler = api_i2c_diag_handler;
    httpd_register_uri_handler(s_camera_httpd, &i2c_diag_uri);

    httpd_uri_t i2c_recover_uri = {};
    i2c_recover_uri.uri = "/api/i2c_recover";
    i2c_recover_uri.method = HTTP_GET;
    i2c_recover_uri.handler = api_i2c_recover_handler;
    httpd_register_uri_handler(s_camera_httpd, &i2c_recover_uri);

    httpd_uri_t files_uri = {};
    files_uri.uri = "/api/files";
    files_uri.method = HTTP_GET;
    files_uri.handler = api_files_handler;
    httpd_register_uri_handler(s_camera_httpd, &files_uri);

    httpd_uri_t file_uri = {};
    file_uri.uri = "/api/file";
    file_uri.method = HTTP_GET;
    file_uri.handler = api_file_handler;
    httpd_register_uri_handler(s_camera_httpd, &file_uri);

    httpd_uri_t fs_uri = {};
    fs_uri.uri = "/api/fs";
    fs_uri.method = HTTP_POST;
    fs_uri.handler = api_fs_handler;
    httpd_register_uri_handler(s_camera_httpd, &fs_uri);

    httpd_uri_t measure_uri = {};
    measure_uri.uri = "/api/measure";
    measure_uri.method = HTTP_GET;
    measure_uri.handler = api_measure_handler;
    httpd_register_uri_handler(s_camera_httpd, &measure_uri);

    httpd_uri_t capture_uri = {};
    capture_uri.uri = "/api/capture";
    capture_uri.method = HTTP_GET;
    capture_uri.handler = api_capture_handler;
    httpd_register_uri_handler(s_camera_httpd, &capture_uri);

    httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
    stream_config.server_port = 81;
    stream_config.ctrl_port = 32769;
    stream_config.max_open_sockets = 1;
    stream_config.max_uri_handlers = 2;
    stream_config.stack_size = 4096;
    stream_config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    stream_config.lru_purge_enable = true;
    s_stream_stop_requested = false;
    esp_err_t stream_err = httpd_start(&s_stream_httpd, &stream_config);
    if (stream_err != ESP_OK) {
        httpd_stop(s_camera_httpd);
        s_camera_httpd = nullptr;
        return stream_err;
    }
    httpd_uri_t dedicated_stream_uri = {};
    dedicated_stream_uri.uri = "/stream";
    dedicated_stream_uri.method = HTTP_GET;
    dedicated_stream_uri.handler = camera_stream_handler;
    httpd_register_uri_handler(s_stream_httpd, &dedicated_stream_uri);

    log_memory("after HTTP and stream servers start");
    if (!wifi_memory_headroom_ok()) {
        ESP_LOGE("heap", "HTTP server stopped: insufficient internal-RAM headroom");
        s_stream_stop_requested = true;
        httpd_stop(s_stream_httpd);
        s_stream_httpd = nullptr;
        httpd_stop(s_camera_httpd);
        s_camera_httpd = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

// 正常模式下的设备文件管理 HTTP 服务:仅注册 SD 文件接口
// (列表/下载/删除/重命名/新建/≤64KiB 文本编辑),不暴露激光、拍照、
// 视频等控制接口(那些仍只在调试 AP 路径 camera_http_start 里注册)。
static esp_err_t device_file_server_start()
{
    if (s_file_httpd) return ESP_OK;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32769;
    config.max_open_sockets = 4;
    config.max_uri_handlers = 4;
    config.stack_size = 12288;
    config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    ESP_RETURN_ON_ERROR(httpd_start(&s_file_httpd, &config), "file_http", "file http start");

    httpd_uri_t files_uri = {};
    files_uri.uri = "/api/files";
    files_uri.method = HTTP_GET;
    files_uri.handler = api_files_handler;
    httpd_register_uri_handler(s_file_httpd, &files_uri);

    httpd_uri_t file_uri = {};
    file_uri.uri = "/api/file";
    file_uri.method = HTTP_GET;
    file_uri.handler = api_file_handler;
    httpd_register_uri_handler(s_file_httpd, &file_uri);

    httpd_uri_t fs_uri = {};
    fs_uri.uri = "/api/fs";
    fs_uri.method = HTTP_POST;
    fs_uri.handler = api_fs_handler;
    httpd_register_uri_handler(s_file_httpd, &fs_uri);

    httpd_uri_t state_uri = {};
    state_uri.uri = "/api/state";
    state_uri.method = HTTP_GET;
    state_uri.handler = api_state_handler;
    httpd_register_uri_handler(s_file_httpd, &state_uri);

    ESP_LOGI("file_http", "device file manager HTTP ready on port 80 (STA mode)");
    return ESP_OK;
}

static esp_err_t wifi_start_camera_ap()
{
    ESP_RETURN_ON_ERROR(wifi_base_init_once(), "cam_ap", "wifi base");
    if (!s_wifi_ap_netif) {
        s_wifi_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_config_t ap_config = {};
    const char *ssid = "ESP32-CAM-TEST";
    const char *pass = "12345678";
    strncpy(reinterpret_cast<char *>(ap_config.ap.ssid), ssid, sizeof(ap_config.ap.ssid));
    strncpy(reinterpret_cast<char *>(ap_config.ap.password), pass, sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len = strlen(ssid);
    ap_config.ap.channel = 6;
    ap_config.ap.max_connection = 2;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.pmf_cfg.required = false;

    log_memory("before Wi-Fi radio start");
    ESP_RETURN_ON_ERROR(wifi_configure_and_start(WIFI_MODE_AP, WIFI_IF_AP, &ap_config),
                        "cam_ap", "ap start");
    ESP_RETURN_ON_ERROR(wifi_set_bandwidth_controlled(WIFI_IF_AP, WIFI_BW_HT20),
                        "cam_ap", "set bandwidth");
    log_memory("after Wi-Fi radio start");
    if (!wifi_memory_headroom_ok()) {
        wifi_stop_if_started();
        ESP_LOGE("heap", "Wi-Fi stopped: insufficient internal-RAM headroom");
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(wifi_set_max_tx_power_controlled(WIFI_AP_MAX_TX_POWER_QDBM),
                        "cam_ap", "set 8 dBm tx power");
    return ESP_OK;
}

static bool camera_init_for_device()
{
    if (s_camera_http_ready) return true;

    I2cLineDiag boot_i2c = {};
    for (int attempt = 0; attempt < 4; ++attempt) {
        boot_i2c = i2c_line_diag(true);
        if (boot_i2c.idle_sda == 1 && boot_i2c.idle_scl == 1) {
            break;
        }
        info("camera_http_init", "waiting for I2C idle before camera init: " + i2c_line_detail(boot_i2c));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (boot_i2c.idle_sda == 1 && boot_i2c.idle_scl == 1) {
        err = camera_http_init();
    } else {
        std::string detail = "camera init skipped; local UI will continue without video: " + i2c_line_detail(boot_i2c);
        fail("camera_http_init", detail);
        dashboard_set_error(detail);
    }

    if (err != ESP_OK) {
        if (err != ESP_ERR_INVALID_STATE) {
            std::string detail = "camera init failed; local UI will continue without video: " + esp_err_str(err);
            fail("camera_http_init", detail);
            dashboard_set_error(detail);
        }
        return false;
    }
    pass("camera_local", "OV5640 ready for local preview; Wi-Fi remains off until WEB menu is opened");
    return true;
}

static bool cmd_start_camera_ap()
{
    if (s_wifi_started && s_camera_httpd && s_stream_httpd) {
        pass("start_camera_ap", "web already online at http://192.168.4.1/");
        return true;
    }
    esp_err_t err = wifi_start_camera_ap();
    if (err != ESP_OK) {
        fail("start_camera_ap", "Wi-Fi AP failed: " + esp_err_str(err));
        return false;
    }
    err = camera_http_start();
    if (err != ESP_OK) {
        wifi_stop_if_started();
        fail("start_camera_ap", "HTTP server failed: " + esp_err_str(err));
        return false;
    }
    sensor_runtime_start();
    if (s_camera_http_ready) {
        pass("start_camera_ap", "SSID=ESP32-CAM-TEST password=12345678 url=http://192.168.4.1/ stream=/stream jpg=/jpg");
    } else {
        pass("start_camera_ap", "SSID=ESP32-CAM-TEST password=12345678 url=http://192.168.4.1/ camera=offline");
    }
    return true;
}

static bool cmd_stop_camera_ap()
{
    if (s_camera_httpd) {
        httpd_stop(s_camera_httpd);
        s_camera_httpd = nullptr;
    }
    if (s_stream_httpd) {
        s_stream_stop_requested = true;
        httpd_stop(s_stream_httpd);
        s_stream_httpd = nullptr;
    }
    wifi_stop_if_started();
    pass("stop_camera_ap", "web and Wi-Fi stopped; local camera preview remains available");
    return true;
}

static bool dashboard_toggle_web()
{
    if (s_web_busy) return false;
    const bool enable = !s_wifi_started || !s_pc_link_enabled.load(std::memory_order_acquire);
    s_web_busy = true;
    s_pc_link_enabled.store(enable, std::memory_order_release);
    if (!enable) s_pc_link_connected.store(false, std::memory_order_release);
    if (s_pc_link_task) xTaskNotifyGive(s_pc_link_task);
    s_web_busy = false;
    dashboard_log_event("pc_link", enable ? "enabled from UI" : "disabled from UI");
    return true;
}

static bool cmd_test_wifi()
{
    if (s_camera_httpd) {
        fail("test_wifi", "camera AP is running; use stop_camera_ap first");
        return false;
    }
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t flash_size = 0;
    esp_flash_get_size(nullptr, &flash_size);
    size_t psram_size = esp_psram_is_initialized() ? esp_psram_get_size() : 0;
    info("test_wifi", "chip_cores=" + std::to_string(chip.cores) + " flash=" + std::to_string(flash_size / 1024 / 1024) + "MB psram=" + std::to_string(psram_size / 1024 / 1024) + "MB");

    esp_err_t err = wifi_init_once();
    if (err != ESP_OK) {
        fail("test_wifi", "wifi init failed: " + esp_err_str(err));
        return false;
    }
    wifi_scan_config_t scan = {};
    err = esp_wifi_scan_start(&scan, true);
    if (err != ESP_OK) {
        fail("test_wifi", "scan failed: " + esp_err_str(err));
        return false;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    std::vector<wifi_ap_record_t> aps(std::min<uint16_t>(n, 12));
    uint16_t getn = aps.size();
    if (getn) {
        esp_wifi_scan_get_ap_records(&getn, aps.data());
    }
    for (int i = 0; i < getn; ++i) {
        info("test_wifi", "ssid=" + std::string(reinterpret_cast<char *>(aps[i].ssid)) + " rssi=" + std::to_string(aps[i].rssi));
    }
    bool ok = psram_size > 0 && n > 0;
    ok ? pass("test_wifi", "PSRAM OK, AP count=" + std::to_string(n))
       : fail("test_wifi", "psram_mb=" + std::to_string(psram_size / 1024 / 1024) + " ap_count=" + std::to_string(n));
    return ok;
}

static void cmd_test_all()
{
    s_summary.clear();
    cmd_scan_i2c();
    cmd_test_pca9557();
    cmd_test_keys();
    cmd_test_bat_adc();
    cmd_test_sd();
    cmd_test_lcd();
    cmd_test_touch();
    cmd_test_bno086();
    cmd_test_laser_once();
    cmd_test_laser_cont();
    cmd_test_camera();
    cmd_test_wifi();
    int pass_count = 0;
    printf("\nSummary:\n");
    for (const auto &r : s_summary) {
        printf("  [%s] %s - %s\n", r.pass ? "PASS" : "FAIL", r.name.c_str(), r.detail.c_str());
        pass_count += r.pass ? 1 : 0;
    }
    char b[80];
    snprintf(b, sizeof(b), "pass=%d total=%u", pass_count, static_cast<unsigned>(s_summary.size()));
    log_line(pass_count == static_cast<int>(s_summary.size()) ? "PASS" : "FAIL", "test_all", b, false);
}

static std::string trim(std::string s)
{
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

static void dispatch(const std::string &cmd)
{
    if (cmd.empty()) return;
    if (cmd == "help") print_help();
    else if (cmd == "pinmap") cmd_pinmap();
    else if (cmd == "test_power_hint") cmd_power_hint();
    else if (cmd == "diag_i2c_lines") cmd_diag_i2c_lines();
    else if (cmd == "recover_i2c") cmd_recover_i2c();
    else if (cmd == "scan_i2c") cmd_scan_i2c();
    else if (cmd == "test_pca9557") cmd_test_pca9557();
    else if (cmd == "test_keys") cmd_test_keys();
    else if (cmd == "test_lcd") cmd_test_lcd();
    else if (cmd == "test_touch") cmd_test_touch();
    else if (cmd == "test_sd") cmd_test_sd();
    else if (cmd == "test_sd_speed") cmd_test_sd_speed();
    else if (cmd == "test_camera") cmd_test_camera();
    else if (cmd == "capture_dataset") {
        std::string path;
        const bool ok = dashboard_capture_photo(&path, "dataset", true);
        ok ? pass("capture_dataset", "high-resolution JPEG saved: " + path)
           : fail("capture_dataset", "capture failed; check camera/SD status and preceding log");
    }
    else if (cmd == "start_camera_ap") cmd_start_camera_ap();
    else if (cmd == "stop_camera_ap") cmd_stop_camera_ap();
    else if (cmd == "stream_camera_uart" || cmd.rfind("stream_camera_uart ", 0) == 0) cmd_stream_camera_uart(cmd);
    else if (cmd == "test_bno086") cmd_test_bno086();
    else if (cmd == "measure_laser") {
        bool ok = dashboard_laser_measure_once();
        DashboardState snap;
        dashboard_lock();
        snap = s_dash;
        dashboard_unlock();
        ok ? pass("measure_laser", "distance_mm=" + std::to_string(snap.laser_mm) + " raw=" + snap.laser_raw)
           : fail("measure_laser", std::string("no valid distance raw=") + snap.laser_raw);
    }
    else if (cmd == "test_laser_once") cmd_test_laser_once();
    else if (cmd == "test_laser_cont") cmd_test_laser_cont();
    else if (cmd == "test_bat_adc") cmd_test_bat_adc();
    else if (cmd == "test_wifi") cmd_test_wifi();
    else if (cmd == "test_all") cmd_test_all();
    else if (cmd == "runtime_status") cmd_runtime_status();
    else if (cmd == "stop_laser" || cmd == "ihalt") cmd_stop_laser();
    else if (cmd == "ui_camera") { device_ui_show_camera(); info("ui", "camera page selected"); }
    else if (cmd == "ui_single") { device_ui_show_single(); info("ui", "single measurement page selected"); }
    else if (cmd == "ui_p2p") { device_ui_show_p2p(); info("ui", "P2P measurement page selected; laser remains off"); }
    else if (cmd == "ui_menu") { device_ui_show_menu(); info("ui", "menu page selected"); }
    else if (cmd == "calibrate_imu") { device_ui_show_imu_calibration(); info("imu_cal", "guided calibration started; use OK to confirm each pose"); }
    else if (cmd == "imu_cal_clear") cmd_imu_cal_clear();
    else if (cmd == "imu_cal_dump") cmd_imu_cal_dump();
    else if (cmd == "room_dump") cmd_room_dump();
    else if (cmd == "room_export_last") cmd_room_export_last();
    else if (cmd == "pc_status") cmd_pc_status();
    else if (cmd == "pc_unbind") cmd_pc_unbind();
    else if (cmd == "reset_pose") {
        s_fusion.resetPose();
        FusionState fused = s_fusion.snapshot();
        dashboard_lock();
        s_dash.path_x = fused.x; s_dash.path_y = fused.y; s_dash.path_z = fused.z;
        s_dash.fusion_vx = fused.vx; s_dash.fusion_vy = fused.vy;
        dashboard_unlock();
        pass("reset_pose", "local X/Y origin and velocity reset");
    }
    else if (cmd == "reset_points") {
        s_fusion.resetMeasurementPair();
        dashboard_lock();
        s_dash.measure_session_id = 0; s_dash.measure_point_count = 0; s_dash.point_distance_m = -1.0f;
        dashboard_unlock();
        pass("reset_points", "P1/P2 measurement pair cleared");
    }
    else {
        fail("command", "unknown command: " + cmd + " ; type help");
    }
}

extern "C" void app_main(void)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("\nESP32-S3 board_self_test @ 115200\n");
    printf("Reset reason: %d\n", static_cast<int>(esp_reset_reason()));
    printf("Type 'help' for commands. Safety: do not aim the L1 laser at eyes.\n");
    // The L1 defaults its emitter on when powered.  Initialize its UART before
    // the LCD/camera checks and the one-second startup delay, then stop it
    // immediately.  sensor_runtime_start() repeats the stop later as a safety
    // net, but the first command must not wait for the runtime tasks.
    if (!s_laser_mutex) s_laser_mutex = xSemaphoreCreateMutex();
    const esp_err_t early_laser_uart_err = uart_init_once(
        LASER_UART_NUM, PIN_LASER_TX, PIN_LASER_RX, LASER_BAUD, &s_uart_laser_ready);
    if (early_laser_uart_err == ESP_OK) {
        const bool early_laser_off = dashboard_laser_stop_continuous(false);
        ESP_LOGI("laser_mode", "early boot initialization: laser %s",
                 early_laser_off ? "OFF" : "STOP FAILED");
    } else {
        ESP_LOGE("laser_mode", "early boot UART initialization failed: %s",
                 esp_err_to_name(early_laser_uart_err));
    }
    i2c_gpio_bus_recover("boot_i2c");
    const esp_err_t boot_i2c_err = i2c_init_once();
    // LCD 自检:初始化 + 彩条(boot 诊断显示,时间缩短;UI 启动后自检页接管)
    bool boot_lcd_ok = false;
    if (nv3030b_lcd_init() == ESP_OK) {
        boot_lcd_ok = nv3030b_lcd_show_test_pattern() == ESP_OK;
    }
    s_dash.check_i2c = boot_i2c_err == ESP_OK ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    s_dash.check_lcd = boot_lcd_ok ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    print_help();
    printf("Starting local device mode; Wi-Fi is available from the WEB menu.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    const bool boot_camera_ok = camera_init_for_device();
    s_dash.check_camera = boot_camera_ok ? DeviceCheckState::PASS : DeviceCheckState::FAIL;
    sensor_runtime_start();

    char line[96];
    printf("selftest> ");
    fflush(stdout);
    while (true) {
        if (fgets(line, sizeof(line), stdin)) {
            std::string cmd = trim(line);
            dispatch(cmd);
            if (!cmd.empty()) {
                printf("selftest> ");
                fflush(stdout);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        process_pending_pc_binding_save();
        process_pending_pc_unbind();
        process_pending_settings_save();
    }
}
