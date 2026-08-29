#include "single_distance.h"

#include <algorithm>
#include <cstring>

void SingleDistanceSession::enter(bool camera_ready, bool laser_uart_ready, int64_t now_us)
{
    data_.page_active = true;
    data_.state = SingleDistanceState::ENTERING;
    data_.error = SingleDistanceError::NONE;
    data_.aim_deadline_us = 0;
    data_.measure_request_us = 0;
    clear_current_result();
    if (!camera_ready) {
        data_.state = SingleDistanceState::INIT_ERROR;
        data_.error = SingleDistanceError::CAMERA_UNAVAILABLE;
    } else if (!laser_uart_ready) {
        data_.state = SingleDistanceState::INIT_ERROR;
        data_.error = SingleDistanceError::LASER_UNAVAILABLE;
    } else {
        data_.state = SingleDistanceState::READY;
    }
    (void)now_us;
}

SingleDistanceAction SingleDistanceSession::exit()
{
    const bool measuring = data_.state == SingleDistanceState::MEASURING;
    const bool laser_on = data_.state == SingleDistanceState::AIMING || measuring;
    data_.page_active = false;
    data_.state = SingleDistanceState::READY;
    data_.error = SingleDistanceError::NONE;
    data_.aim_deadline_us = 0;
    data_.measure_request_us = 0;
    clear_current_result();
    if (measuring) return SingleDistanceAction::CANCEL_MEASUREMENT;
    return laser_on ? SingleDistanceAction::LASER_OFF : SingleDistanceAction::NONE;
}

SingleDistanceAction SingleDistanceSession::begin_aiming(int64_t now_us)
{
    data_.state = SingleDistanceState::AIMING;
    data_.error = SingleDistanceError::NONE;
    data_.measurement_reference = data_.selected_reference;
    data_.aim_deadline_us = now_us + kAimTimeoutUs;
    data_.measure_request_us = 0;
    clear_current_result();
    return SingleDistanceAction::LASER_ON;
}

SingleDistanceAction SingleDistanceSession::trigger(int64_t now_us)
{
    if (!data_.page_active) return SingleDistanceAction::NONE;
    switch (data_.state) {
    case SingleDistanceState::READY:
    case SingleDistanceState::RESULT:
    case SingleDistanceState::SAVED:
    case SingleDistanceState::MEASURE_ERROR:
        return begin_aiming(now_us);
    case SingleDistanceState::AIMING:
        data_.state = SingleDistanceState::MEASURING;
        data_.aim_deadline_us = 0;
        data_.measure_request_us = now_us;
        return SingleDistanceAction::START_MEASUREMENT;
    default:
        return SingleDistanceAction::NONE;
    }
}

SingleDistanceAction SingleDistanceSession::cancel()
{
    switch (data_.state) {
    case SingleDistanceState::AIMING:
        data_.state = SingleDistanceState::READY;
        data_.aim_deadline_us = 0;
        return SingleDistanceAction::LASER_OFF;
    case SingleDistanceState::MEASURING:
        data_.state = SingleDistanceState::READY;
        data_.measure_request_us = 0;
        return SingleDistanceAction::CANCEL_MEASUREMENT;
    case SingleDistanceState::RESULT:
    case SingleDistanceState::SAVED:
        data_.state = SingleDistanceState::READY;
        clear_current_result();
        return SingleDistanceAction::NONE;
    case SingleDistanceState::SAVE_ERROR:
        data_.state = SingleDistanceState::RESULT;
        data_.error = SingleDistanceError::NONE;
        return SingleDistanceAction::NONE;
    case SingleDistanceState::MEASURE_ERROR:
    case SingleDistanceState::INIT_ERROR:
        data_.state = SingleDistanceState::READY;
        data_.error = SingleDistanceError::NONE;
        return SingleDistanceAction::NONE;
    default:
        return SingleDistanceAction::NONE;
    }
}

SingleDistanceAction SingleDistanceSession::tick(int64_t now_us)
{
    if (data_.state == SingleDistanceState::AIMING && data_.aim_deadline_us > 0 &&
        now_us >= data_.aim_deadline_us) {
        data_.state = SingleDistanceState::READY;
        data_.aim_deadline_us = 0;
        data_.error = SingleDistanceError::NONE;
        return SingleDistanceAction::LASER_OFF;
    }
    return SingleDistanceAction::NONE;
}

bool SingleDistanceSession::cycle_reference()
{
    if (!data_.page_active || data_.state != SingleDistanceState::READY) return false;
    const uint8_t next = (static_cast<uint8_t>(data_.selected_reference) + 1u) % 3u;
    data_.selected_reference = static_cast<DistanceReference>(next);
    return true;
}

bool SingleDistanceSession::measurement_succeeded(const SingleDistanceResult &result)
{
    if (data_.state != SingleDistanceState::MEASURING || result.distance_mm <= 0) return false;
    data_.result = result;
    data_.result.temporary_id = next_temporary_id_++;
    data_.result.reference = data_.measurement_reference;
    data_.result.saved_record_id = 0;
    data_.result_valid = true;
    data_.state = SingleDistanceState::RESULT;
    data_.error = SingleDistanceError::NONE;
    data_.measure_request_us = 0;

    const size_t move_count = std::min<size_t>(data_.history_count,
                                               SINGLE_DISTANCE_HISTORY_CAPACITY - 1);
    if (move_count > 0) {
        memmove(&data_.history[1], &data_.history[0], move_count * sizeof(data_.history[0]));
    }
    data_.history[0] = data_.result;
    if (data_.history_count < SINGLE_DISTANCE_HISTORY_CAPACITY) ++data_.history_count;
    return true;
}

bool SingleDistanceSession::measurement_failed(SingleDistanceError error, int32_t raw_error_code)
{
    if (data_.state != SingleDistanceState::MEASURING &&
        data_.state != SingleDistanceState::AIMING) return false;
    data_.state = SingleDistanceState::MEASURE_ERROR;
    data_.error = error == SingleDistanceError::NONE ? SingleDistanceError::NO_RETURN_SIGNAL : error;
    data_.result.raw_error_code = raw_error_code;
    data_.aim_deadline_us = 0;
    data_.measure_request_us = 0;
    data_.result_valid = false;
    return true;
}

bool SingleDistanceSession::request_save()
{
    if (!data_.result_valid ||
        (data_.state != SingleDistanceState::RESULT && data_.state != SingleDistanceState::SAVE_ERROR)) {
        return false;
    }
    data_.state = SingleDistanceState::SAVING;
    data_.error = SingleDistanceError::NONE;
    return true;
}

bool SingleDistanceSession::save_succeeded(uint32_t record_id)
{
    if (data_.state != SingleDistanceState::SAVING || record_id == 0) return false;
    data_.state = SingleDistanceState::SAVED;
    data_.error = SingleDistanceError::NONE;
    data_.result.saved_record_id = record_id;
    for (size_t i = 0; i < data_.history_count; ++i) {
        if (data_.history[i].temporary_id == data_.result.temporary_id) {
            data_.history[i].saved_record_id = record_id;
            break;
        }
    }
    // A successful save completes the transaction. Return immediately to a
    // fresh live-preview state instead of leaving the frozen result onscreen.
    data_.state = SingleDistanceState::READY;
    clear_current_result();
    return true;
}

bool SingleDistanceSession::save_failed(SingleDistanceError error, int32_t raw_error_code)
{
    if (data_.state != SingleDistanceState::SAVING) return false;
    data_.state = SingleDistanceState::SAVE_ERROR;
    data_.error = error == SingleDistanceError::NONE ? SingleDistanceError::SAVE_FAILED : error;
    data_.result.raw_error_code = raw_error_code;
    return true;
}

SingleDistanceSnapshot SingleDistanceSession::snapshot() const
{
    return data_;
}

void SingleDistanceSession::clear_current_result()
{
    data_.result_valid = false;
    data_.result = {};
}
