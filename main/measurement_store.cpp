#include "measurement_store.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

constexpr uint32_t kV2RecordIdBase = 1000000;
constexpr uint32_t kV3RecordIdBase = 2000000;

SemaphoreHandle_t semaphore(void *handle)
{
    return static_cast<SemaphoreHandle_t>(handle);
}

class ScopedLock {
public:
    explicit ScopedLock(void *mutex, TickType_t timeout = pdMS_TO_TICKS(2000))
        : mutex_(semaphore(mutex)), locked_(mutex_ && xSemaphoreTake(mutex_, timeout) == pdTRUE)
    {
    }

    ~ScopedLock()
    {
        if (locked_) xSemaphoreGive(mutex_);
    }

    bool locked() const { return locked_; }

private:
    SemaphoreHandle_t mutex_ = nullptr;
    bool locked_ = false;
};

std::vector<std::string> split_csv_line(const char *line)
{
    std::vector<std::string> fields;
    if (!line) return fields;
    const char *start = line;
    for (const char *p = line;; ++p) {
        if (*p == ',' || *p == '\0' || *p == '\r' || *p == '\n') {
            fields.emplace_back(start, static_cast<size_t>(p - start));
            if (*p != ',') break;
            start = p + 1;
        }
    }
    return fields;
}

std::string safe_image_path(const char *path)
{
    if (!path || path[0] == '\0') return "-";
    std::string clean(path);
    for (char &c : clean) {
        if (c == ',' || c == '\r' || c == '\n') c = '_';
    }
    return clean;
}

}  // namespace

esp_err_t MeasurementStore::begin(const char *mount_point)
{
    if (!mount_point || mount_point[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (!mutex_) {
        mutex_ = xSemaphoreCreateMutex();
        if (!mutex_) return ESP_ERR_NO_MEM;
    }

    ScopedLock lock(mutex_);
    if (!lock.locked()) return ESP_ERR_TIMEOUT;

    const std::string legacy_path = std::string(mount_point) + "/measurement_records.csv";
    const std::string v2_path = std::string(mount_point) + "/measurement_records_v2.csv";
    path_ = std::string(mount_point) + "/measurement_records_v3.csv";
    records_.clear();
    records_.reserve(32);
    next_id_ = 1;
    truncated_ = false;
    ready_ = false;

    // V1 rows remain readable, but all new events go to a V2 file with an
    // honest extended header. Replaying legacy first also lets V2 tombstones
    // delete old records without rewriting either file.
    auto replay_file = [&](const std::string &replay_path) {
        FILE *replay = fopen(replay_path.c_str(), "r");
        if (!replay) return;
        char line[512] = {};
        while (fgets(line, sizeof(line), replay)) {
            const std::vector<std::string> fields = split_csv_line(line);
            if (fields.size() < 4 || (fields[0] != "SAVE" && fields[0] != "DELETE")) continue;
            const uint32_t id = static_cast<uint32_t>(strtoul(fields[1].c_str(), nullptr, 10));
            if (id == 0) continue;
            next_id_ = std::max(next_id_, id + 1);
            if (fields[0] == "SAVE") {
                MeasurementRecord record;
                record.id = id;
                record.t_us = static_cast<int64_t>(strtoll(fields[2].c_str(), nullptr, 10));
                record.distance_mm = static_cast<int32_t>(strtol(fields[3].c_str(), nullptr, 10));
                if (fields.size() >= 13) {
                    const unsigned long reference = strtoul(fields[4].c_str(), nullptr, 10);
                    if (reference <= static_cast<unsigned long>(DistanceReference::TRIPOD)) {
                        record.reference = static_cast<DistanceReference>(reference);
                    }
                    record.pitch_deg = strtof(fields[5].c_str(), nullptr);
                    record.roll_deg = strtof(fields[6].c_str(), nullptr);
                    record.yaw_deg = strtof(fields[7].c_str(), nullptr);
                    record.camera_frame_saved = strtoul(fields[8].c_str(), nullptr, 10) != 0;
                    if (fields[9] != "-") {
                        snprintf(record.image_path, sizeof(record.image_path), "%s", fields[9].c_str());
                    }
                    record.quality = static_cast<uint8_t>(strtoul(fields[10].c_str(), nullptr, 10));
                    record.measure_time_ms = static_cast<uint16_t>(strtoul(fields[11].c_str(), nullptr, 10));
                    record.raw_error_code = static_cast<int32_t>(strtol(fields[12].c_str(), nullptr, 10));
                }
                if (fields.size() >= 27) {
                    const unsigned long type = strtoul(fields[13].c_str(), nullptr, 10);
                    if (type <= static_cast<unsigned long>(MeasurementRecord::Type::FLOORPLAN))
                        record.type = static_cast<MeasurementRecord::Type>(type);
                    record.distance_a_mm = static_cast<int32_t>(strtol(fields[14].c_str(), nullptr, 10));
                    record.distance_b_mm = static_cast<int32_t>(strtol(fields[15].c_str(), nullptr, 10));
                    record.horizontal_mm = static_cast<int32_t>(strtol(fields[16].c_str(), nullptr, 10));
                    record.height_diff_mm = static_cast<int32_t>(strtol(fields[17].c_str(), nullptr, 10));
                    if (fields[18] != "-") snprintf(record.image_path_b, sizeof(record.image_path_b), "%s", fields[18].c_str());
                    record.timestamp_a_ms = static_cast<int64_t>(strtoll(fields[19].c_str(), nullptr, 10));
                    record.timestamp_b_ms = static_cast<int64_t>(strtoll(fields[20].c_str(), nullptr, 10));
                    record.pitch_a_deg = strtof(fields[21].c_str(), nullptr);
                    record.roll_a_deg = strtof(fields[22].c_str(), nullptr);
                    record.yaw_a_deg = strtof(fields[23].c_str(), nullptr);
                    record.pitch_b_deg = strtof(fields[24].c_str(), nullptr);
                    record.roll_b_deg = strtof(fields[25].c_str(), nullptr);
                    record.yaw_b_deg = strtof(fields[26].c_str(), nullptr);
                }
                if (record.distance_mm > 0) apply_save_locked(record);
            } else {
                apply_delete_locked(id);
            }
        }
        fclose(replay);
    };

    replay_file(legacy_path);
    replay_file(v2_path);
    // Keep V2 IDs disjoint from IDs an older firmware could append to the V1
    // file after a downgrade, avoiding cross-file record replacement.
    next_id_ = std::max(next_id_, kV3RecordIdBase);
    FILE *file = fopen(path_.c_str(), "a+");
    if (!file) return ESP_FAIL;
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return ESP_FAIL;
    }
    if (ftell(file) == 0) {
        if (fputs("op,id,t_us,distance_mm,reference,pitch_deg,roll_deg,yaw_deg,"
                  "camera_frame_saved,image_path,quality,measure_time_ms,raw_error_code,"
                  "type,distance_a_mm,distance_b_mm,horizontal_mm,height_diff_mm,image_path_b,"
                  "timestamp_a_ms,timestamp_b_ms,pitch_a_deg,roll_a_deg,yaw_a_deg,"
                  "pitch_b_deg,roll_b_deg,yaw_b_deg\n", file) < 0 ||
            fflush(file) != 0 ||
            fsync(fileno(file)) != 0) {
            fclose(file);
            return ESP_FAIL;
        }
    }
    fclose(file);
    replay_file(path_);
    ready_ = true;
    return ESP_OK;
}

esp_err_t MeasurementStore::append_event_locked(const char *operation, const MeasurementRecord &record)
{
    FILE *file = fopen(path_.c_str(), "a");
    if (!file) return ESP_FAIL;
    const std::string image_path = safe_image_path(record.image_path);
    const std::string image_path_b = safe_image_path(record.image_path_b);
    const int result = fprintf(file,
                               "%s,%lu,%lld,%ld,%u,%.3f,%.3f,%.3f,%u,%s,%u,%u,%ld,"
                               "%u,%ld,%ld,%ld,%ld,%s,%lld,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                               operation,
                               static_cast<unsigned long>(record.id),
                               static_cast<long long>(record.t_us),
                               static_cast<long>(record.distance_mm),
                               static_cast<unsigned>(record.reference),
                               static_cast<double>(record.pitch_deg),
                               static_cast<double>(record.roll_deg),
                               static_cast<double>(record.yaw_deg),
                               record.camera_frame_saved ? 1u : 0u,
                               image_path.c_str(),
                               static_cast<unsigned>(record.quality),
                               static_cast<unsigned>(record.measure_time_ms),
                               static_cast<long>(record.raw_error_code),
                               static_cast<unsigned>(record.type),
                               static_cast<long>(record.distance_a_mm),
                               static_cast<long>(record.distance_b_mm),
                               static_cast<long>(record.horizontal_mm),
                               static_cast<long>(record.height_diff_mm),
                               image_path_b.c_str(),
                               static_cast<long long>(record.timestamp_a_ms),
                               static_cast<long long>(record.timestamp_b_ms),
                               static_cast<double>(record.pitch_a_deg),
                               static_cast<double>(record.roll_a_deg),
                               static_cast<double>(record.yaw_a_deg),
                               static_cast<double>(record.pitch_b_deg),
                               static_cast<double>(record.roll_b_deg),
                               static_cast<double>(record.yaw_b_deg));
    bool ok = result > 0 && fflush(file) == 0 && fsync(fileno(file)) == 0;
    fclose(file);
    return ok ? ESP_OK : ESP_FAIL;
}

void MeasurementStore::apply_save_locked(const MeasurementRecord &record)
{
    auto existing = std::find_if(records_.begin(), records_.end(),
                                 [&](const MeasurementRecord &item) { return item.id == record.id; });
    if (existing != records_.end()) {
        *existing = record;
        return;
    }
    if (records_.size() >= kMaxActiveRecords) {
        records_.erase(records_.begin());
        truncated_ = true;
    }
    records_.push_back(record);
}

bool MeasurementStore::apply_delete_locked(uint32_t id)
{
    auto existing = std::find_if(records_.begin(), records_.end(),
                                 [&](const MeasurementRecord &item) { return item.id == id; });
    if (existing == records_.end()) return false;
    records_.erase(existing);
    return true;
}

esp_err_t MeasurementStore::append(const MeasurementRecord &draft, MeasurementRecord *saved)
{
    if (!ready_) return ESP_ERR_INVALID_STATE;
    if (draft.distance_mm <= 0) return ESP_ERR_INVALID_ARG;
    ScopedLock lock(mutex_);
    if (!lock.locked()) return ESP_ERR_TIMEOUT;

    MeasurementRecord record = draft;
    record.id = next_id_;
    esp_err_t err = append_event_locked("SAVE", record);
    if (err != ESP_OK) return err;
    ++next_id_;
    apply_save_locked(record);
    if (saved) *saved = record;
    return ESP_OK;
}

esp_err_t MeasurementStore::erase(uint32_t id, int64_t t_us)
{
    if (!ready_) return ESP_ERR_INVALID_STATE;
    if (id == 0) return ESP_ERR_INVALID_ARG;
    ScopedLock lock(mutex_);
    if (!lock.locked()) return ESP_ERR_TIMEOUT;

    const bool found = std::any_of(records_.begin(), records_.end(),
                                   [&](const MeasurementRecord &item) { return item.id == id; });
    if (!found) return truncated_ ? ESP_ERR_NOT_SUPPORTED : ESP_ERR_NOT_FOUND;
    MeasurementRecord tombstone = {id, t_us, 0};
    esp_err_t err = append_event_locked("DELETE", tombstone);
    if (err != ESP_OK) return err;
    apply_delete_locked(id);
    return ESP_OK;
}

bool MeasurementStore::find(uint32_t id, MeasurementRecord *record)
{
    if (!record || !ready_ || id == 0) return false;
    ScopedLock lock(mutex_);
    if (!lock.locked()) return false;
    const auto found = std::find_if(records_.begin(), records_.end(),
                                    [&](const MeasurementRecord &item) { return item.id == id; });
    if (found == records_.end()) return false;
    *record = *found;
    return true;
}

size_t MeasurementStore::snapshot_newest(MeasurementRecord *records, size_t capacity, size_t *total)
{
    if (!mutex_) {
        if (total) *total = 0;
        return 0;
    }
    ScopedLock lock(mutex_);
    if (!lock.locked()) {
        if (total) *total = 0;
        return 0;
    }
    if (total) *total = records_.size();
    const size_t count = std::min(capacity, records_.size());
    for (size_t i = 0; i < count; ++i) {
        records[i] = records_[records_.size() - 1 - i];
    }
    return count;
}
