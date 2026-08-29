#pragma once

#include <stddef.h>
#include <stdint.h>

enum class DistanceReference : uint8_t {
    REAR = 0,
    FRONT = 1,
    TRIPOD = 2,
};

enum class SingleDistanceState : uint8_t {
    ENTERING = 0,
    READY,
    AIMING,
    MEASURING,
    RESULT,
    MEASURE_ERROR,
    SAVING,
    SAVED,
    SAVE_ERROR,
    INIT_ERROR,
};

enum class SingleDistanceError : int32_t {
    NONE = 0,
    CAMERA_UNAVAILABLE = 1,
    LASER_UNAVAILABLE = 2,
    LASER_BUSY = 3,
    NO_RETURN_SIGNAL = 4,
    MEASURE_TIMEOUT = 5,
    STORAGE_UNAVAILABLE = 6,
    SAVE_FAILED = 7,
};

enum class SingleDistanceAction : uint8_t {
    NONE = 0,
    LASER_ON,
    LASER_OFF,
    START_MEASUREMENT,
    CANCEL_MEASUREMENT,
};

struct SingleDistanceResult {
    uint32_t temporary_id = 0;
    int64_t timestamp_ms = 0;
    int32_t distance_mm = 0;
    DistanceReference reference = DistanceReference::REAR;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
    bool camera_frame_saved = false;
    char image_path[96] = "";
    uint8_t quality = 0;
    uint16_t measure_time_ms = 0;
    int32_t raw_error_code = 0;
    uint32_t saved_record_id = 0;
};

static constexpr size_t SINGLE_DISTANCE_HISTORY_CAPACITY = 20;

struct SingleDistanceSnapshot {
    bool page_active = false;
    SingleDistanceState state = SingleDistanceState::ENTERING;
    SingleDistanceError error = SingleDistanceError::NONE;
    DistanceReference selected_reference = DistanceReference::REAR;
    DistanceReference measurement_reference = DistanceReference::REAR;
    int64_t aim_deadline_us = 0;
    int64_t measure_request_us = 0;
    bool result_valid = false;
    SingleDistanceResult result = {};
    uint8_t history_count = 0;
    SingleDistanceResult history[SINGLE_DISTANCE_HISTORY_CAPACITY] = {};
};

// Pure single-distance workflow. Hardware and storage actions are returned to
// the application so this state machine never blocks the UI task.
class SingleDistanceSession {
public:
    static constexpr int64_t kAimTimeoutUs = 15000000;

    void enter(bool camera_ready, bool laser_uart_ready, int64_t now_us);
    SingleDistanceAction exit();
    SingleDistanceAction trigger(int64_t now_us);
    SingleDistanceAction cancel();
    SingleDistanceAction tick(int64_t now_us);
    bool cycle_reference();

    bool measurement_succeeded(const SingleDistanceResult &result);
    bool measurement_failed(SingleDistanceError error, int32_t raw_error_code);
    bool request_save();
    bool save_succeeded(uint32_t record_id);
    bool save_failed(SingleDistanceError error, int32_t raw_error_code);

    SingleDistanceSnapshot snapshot() const;

private:
    SingleDistanceAction begin_aiming(int64_t now_us);
    void clear_current_result();

    SingleDistanceSnapshot data_ = {};
    uint32_t next_temporary_id_ = 1;
};
