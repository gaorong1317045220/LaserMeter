#pragma once

#include <stddef.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "esp_err.h"
#include "single_distance.h"

struct MeasurementRecord {
    enum class Type : uint8_t { SINGLE = 0, P2P = 1, FLOORPLAN = 2 };
    uint32_t id = 0;
    int64_t t_us = 0;
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
    Type type = Type::SINGLE;
    int32_t distance_a_mm = 0;
    int32_t distance_b_mm = 0;
    int32_t horizontal_mm = 0;
    int32_t height_diff_mm = 0;
    char image_path_b[96] = "";
    int64_t timestamp_a_ms = 0;
    int64_t timestamp_b_ms = 0;
    float pitch_a_deg = 0.0f;
    float roll_a_deg = 0.0f;
    float yaw_a_deg = 0.0f;
    float pitch_b_deg = 0.0f;
    float roll_b_deg = 0.0f;
    float yaw_b_deg = 0.0f;
};

// An append-only event log makes both save and delete resilient to power loss:
// deleting a record appends a tombstone instead of rewriting the whole file.
class MeasurementStore {
public:
    esp_err_t begin(const char *mount_point);
    // The caller supplies measurement metadata; the store owns record IDs.
    esp_err_t append(const MeasurementRecord &draft, MeasurementRecord *saved = nullptr);
    esp_err_t erase(uint32_t id, int64_t t_us);
    bool find(uint32_t id, MeasurementRecord *record);
    size_t snapshot_newest(MeasurementRecord *records, size_t capacity, size_t *total = nullptr);
    bool ready() const { return ready_; }
    const char *path() const { return path_.c_str(); }

private:
    static constexpr size_t kMaxActiveRecords = 256;

    esp_err_t append_event_locked(const char *operation, const MeasurementRecord &record);
    void apply_save_locked(const MeasurementRecord &record);
    bool apply_delete_locked(uint32_t id);

    void *mutex_ = nullptr;
    std::string path_;
    std::vector<MeasurementRecord> records_;
    uint32_t next_id_ = 1;
    bool ready_ = false;
    bool truncated_ = false;
};
