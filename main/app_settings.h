#pragma once

#include <stdint.h>

#include "esp_err.h"

enum class DistanceUnit : uint8_t {
    MILLIMETRES = 0,
    METRES = 1,
};

struct AppSettings {
    bool auto_save_single = false;
    bool photo_on_measure = true;
    DistanceUnit distance_unit = DistanceUnit::MILLIMETRES;
};

AppSettings app_settings_defaults();
esp_err_t app_settings_load(AppSettings *settings);
esp_err_t app_settings_save(const AppSettings &settings);
