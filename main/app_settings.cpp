#include "app_settings.h"

#include "nvs.h"

namespace {

constexpr char kNamespace[] = "app_settings";
constexpr char kAutoSaveKey[] = "auto_save";
constexpr char kPhotoKey[] = "photo";
constexpr char kUnitKey[] = "unit";

uint8_t bool_value(bool value)
{
    return value ? 1 : 0;
}

}  // namespace

AppSettings app_settings_defaults()
{
    return AppSettings{};
}

esp_err_t app_settings_load(AppSettings *settings)
{
    if (!settings) return ESP_ERR_INVALID_ARG;
    *settings = app_settings_defaults();

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (err != ESP_OK) return err;

    uint8_t value = 0;
    if (nvs_get_u8(handle, kAutoSaveKey, &value) == ESP_OK) {
        settings->auto_save_single = value != 0;
    }
    if (nvs_get_u8(handle, kPhotoKey, &value) == ESP_OK) {
        settings->photo_on_measure = value != 0;
    }
    if (nvs_get_u8(handle, kUnitKey, &value) == ESP_OK &&
        value <= static_cast<uint8_t>(DistanceUnit::METRES)) {
        settings->distance_unit = static_cast<DistanceUnit>(value);
    }
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t app_settings_save(const AppSettings &settings)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(handle, kAutoSaveKey, bool_value(settings.auto_save_single));
    if (err == ESP_OK) err = nvs_set_u8(handle, kPhotoKey, bool_value(settings.photo_on_measure));
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, kUnitKey, static_cast<uint8_t>(settings.distance_unit));
    }
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

