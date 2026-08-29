#include "nv3030b_lcd.h"

#include <algorithm>
#include <stddef.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "pin_config.h"

namespace {

constexpr size_t kTransferBytes = 32768;
constexpr int kStripeRows = 16;
constexpr uint8_t kWriteCommandOpcode = 0x02;
constexpr uint8_t kRamWriteCommand = 0x2C;

const char *TAG = "nv3030b";
esp_lcd_panel_io_handle_t s_io = nullptr;
uint16_t *s_stripe = nullptr;
SemaphoreHandle_t s_lcd_mutex = nullptr;

struct InitCommand {
    uint8_t command;
    uint8_t length;
    uint16_t delay_ms;
    uint8_t data[8];
};

// TK018F3716 merchant initialization sequence for the NV3030B controller in
// single-data-line SPI mode. Keep this separate from the merchant's QSPI-only
// DE/DF/CE/D8 register-unlock sequence.
constexpr InitCommand kInitCommands[] = {
    {0x01, 0, 120, {0}},
    {0xFF, 3, 0,   {0x20, 0x10, 0x00}},
    {0x36, 1, 0,   {0x08}},
    {0x3A, 1, 0,   {0x55}},
    {0xFF, 3, 0,   {0x20, 0x10, 0x10}},
    {0x0C, 1, 0,   {0x11}},
    {0x10, 1, 0,   {0x02}},
    {0x11, 1, 0,   {0x11}},
    {0x15, 1, 0,   {0x42}},
    {0x16, 1, 0,   {0x11}},
    {0x1A, 1, 0,   {0x02}},
    {0x11, 0, 10,  {0}},
    {0xFD, 2, 0,   {0x06, 0x08}},
    {0x61, 2, 0,   {0x07, 0x04}},
    {0x62, 3, 0,   {0x00, 0x44, 0x45}},
    {0x63, 4, 0,   {0x41, 0x07, 0x12, 0x12}},
    {0x64, 1, 0,   {0x37}},
    {0x65, 3, 0,   {0x09, 0x10, 0x21}},
    {0x66, 3, 0,   {0x09, 0x10, 0x21}},
    {0x67, 2, 0,   {0x20, 0x40}},
    {0x68, 4, 0,   {0x90, 0x4C, 0x7C, 0x66}},
    {0xB1, 3, 0,   {0x0F, 0x02, 0x01}},
    {0xB4, 1, 0,   {0x01}},
    {0xB5, 4, 0,   {0x02, 0x02, 0x0A, 0x14}},
    {0xB6, 5, 0,   {0x04, 0x01, 0x9F, 0x00, 0x02}},
    {0xDF, 1, 0,   {0x11}},
    {0xE2, 6, 0,   {0x13, 0x00, 0x00, 0x30, 0x33, 0x3F}},
    {0xE5, 6, 0,   {0x3F, 0x33, 0x30, 0x00, 0x00, 0x13}},
    {0xE1, 2, 0,   {0x00, 0x57}},
    {0xE4, 2, 0,   {0x58, 0x00}},
    {0xE0, 7, 0,   {0x01, 0x03, 0x0E, 0x0E, 0x0C, 0x15, 0x19}},
    {0xE3, 8, 0,   {0x1A, 0x16, 0x0C, 0x0F, 0x0E, 0x0D, 0x02, 0x01}},
    {0xE6, 2, 0,   {0x00, 0xFF}},
    {0xE7, 6, 0,   {0x01, 0x04, 0x03, 0x03, 0x00, 0x12}},
    {0xE8, 3, 0,   {0x00, 0x70, 0x00}},
    {0xEC, 1, 0,   {0x52}},
    {0xF1, 3, 0,   {0x01, 0x01, 0x02}},
    {0xF6, 4, 0,   {0x09, 0x10, 0x00, 0x00}},
    {0xFD, 2, 0,   {0xFA, 0xFC}},
    {0x3A, 1, 0,   {0x05}},
    {0x36, 1, 0,   {0x08}},
    {0x35, 1, 0,   {0x00}},
    {0x21, 0, 1,   {0}},
    {0x3A, 1, 0,   {0x55}},
    {0x36, 1, 0,   {0x08}},
    {0x11, 0, 120, {0}},
    {0x29, 0, 100, {0}},
};

constexpr int pack_command(uint8_t command)
{
    // The panel's merchant "SPI" mode is one data line, but it still uses
    // the 32-bit command frame [02 00 CMD 00]. It is not D/C + 8-bit SPI.
    return (static_cast<uint32_t>(kWriteCommandOpcode) << 24) |
           (static_cast<uint32_t>(command) << 8);
}

esp_err_t write_command(uint8_t command, const uint8_t *data = nullptr, size_t length = 0)
{
    return esp_lcd_panel_io_tx_param(s_io, pack_command(command), data, length);
}

esp_err_t wait_for_color_transfer()
{
    return esp_lcd_panel_io_tx_param(s_io, -1, nullptr, 0);
}

esp_err_t set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    const uint8_t columns[] = {
        static_cast<uint8_t>(x0 >> 8), static_cast<uint8_t>(x0),
        static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1),
    };
    const uint8_t rows[] = {
        static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0),
        static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1),
    };
    ESP_RETURN_ON_ERROR(write_command(0x2A, columns, sizeof(columns)), TAG, "column window");
    return write_command(0x2B, rows, sizeof(rows));
}

esp_err_t fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    if (!s_io || !s_stripe || x >= LCD_WIDTH || y >= LCD_HEIGHT || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    width = std::min<uint16_t>(width, LCD_WIDTH - x);
    height = std::min<uint16_t>(height, LCD_HEIGHT - y);
    const uint16_t wire_color = __builtin_bswap16(color);

    for (uint16_t row = 0; row < height; row += kStripeRows) {
        const uint16_t rows = std::min<uint16_t>(kStripeRows, height - row);
        std::fill_n(s_stripe, static_cast<size_t>(width) * rows, wire_color);
        ESP_RETURN_ON_ERROR(set_window(x, y + row, x + width - 1, y + row + rows - 1), TAG, "set window");
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_io_tx_color(s_io, pack_command(kRamWriteCommand), s_stripe,
                                      static_cast<size_t>(width) * rows * sizeof(uint16_t)),
            TAG, "write pixels");
        ESP_RETURN_ON_ERROR(wait_for_color_transfer(), TAG, "wait pixels");
    }
    return ESP_OK;
}

esp_err_t present_rect_locked(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                              const uint16_t *pixels, size_t pixel_count)
{
    if (!s_io || !s_stripe || !pixels || width == 0 || height == 0 ||
        x >= LCD_WIDTH || y >= LCD_HEIGHT || x + width > LCD_WIDTH || y + height > LCD_HEIGHT ||
        pixel_count < static_cast<size_t>(width) * height) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint16_t row = 0; row < height; row += kStripeRows) {
        const uint16_t rows = std::min<uint16_t>(kStripeRows, height - row);
        const size_t count = static_cast<size_t>(width) * rows;
        memcpy(s_stripe, pixels + static_cast<size_t>(row) * width, count * sizeof(uint16_t));
        ESP_RETURN_ON_ERROR(set_window(x, y + row, x + width - 1, y + row + rows - 1), TAG, "set frame window");
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_io_tx_color(s_io, pack_command(kRamWriteCommand), s_stripe,
                                      count * sizeof(uint16_t)),
            TAG, "write frame pixels");
        ESP_RETURN_ON_ERROR(wait_for_color_transfer(), TAG, "wait frame pixels");
    }
    return ESP_OK;
}

// Boot brand bitmap (240x92 RGB565) rendered with real Chinese text.
extern "C" const lv_img_dsc_t nv3030b_boot_brand;

esp_err_t show_boot_brand_locked()
{
    esp_err_t err = fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x10A3);  // dark navy 0x10161E
    if (err != ESP_OK) return err;
    err = present_rect_locked(0, 0, 240, 92,
                              reinterpret_cast<const uint16_t *>(nv3030b_boot_brand.data),
                              static_cast<size_t>(240) * 92);
    if (err != ESP_OK) return err;
    err = fill_rect(40, 256, 160, 3, 0x636F);      // progress track
    if (err != ESP_OK) return err;
    return fill_rect(40, 256, 80, 3, 0x331D);      // blue progress ~50%
}

}  // namespace

esp_err_t nv3030b_lcd_show_boot_brand()
{
    if (!s_lcd_mutex || xSemaphoreTake(s_lcd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = show_boot_brand_locked();
    xSemaphoreGive(s_lcd_mutex);
    return err;
}

esp_err_t nv3030b_lcd_init()
{
    if (s_io) {
        return ESP_OK;
    }

    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_LCD_MOSI;
    bus.miso_io_num = GPIO_NUM_NC;
    bus.sclk_io_num = PIN_LCD_SCLK;
    bus.quadwp_io_num = GPIO_NUM_NC;
    bus.quadhd_io_num = GPIO_NUM_NC;
    bus.max_transfer_sz = kTransferBytes;
    bus.flags = SPICOMMON_BUSFLAG_MASTER;
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "SPI bus init");

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = PIN_LCD_CS;
    io_config.dc_gpio_num = GPIO_NUM_NC;
    io_config.spi_mode = 3;
    io_config.pclk_hz = LCD_SPI_CLOCK_HZ;
    // All transfers are awaited synchronously; two descriptors are sufficient
    // and leave more contiguous internal DMA RAM available for Wi-Fi.
    io_config.trans_queue_depth = 2;
    io_config.lcd_cmd_bits = 32;
    io_config.lcd_param_bits = 8;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(LCD_SPI_HOST),
                                 &io_config, &s_io),
        TAG, "panel IO init");

    s_stripe = static_cast<uint16_t *>(
        heap_caps_malloc(LCD_WIDTH * kStripeRows * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    ESP_RETURN_ON_FALSE(s_stripe, ESP_ERR_NO_MEM, TAG, "stripe allocation failed");
    s_lcd_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lcd_mutex, ESP_ERR_NO_MEM, TAG, "LCD mutex allocation failed");

    vTaskDelay(pdMS_TO_TICKS(150));
    for (const auto &entry : kInitCommands) {
        ESP_RETURN_ON_ERROR(write_command(entry.command, entry.data, entry.length), TAG, "panel init command");
        if (entry.delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(entry.delay_ms));
        }
    }
    ESP_LOGI(TAG, "NV3030B ready: %dx%d 1-line SPI mode 3, framed 32-bit commands at %d MHz, CS=%d SCK=%d MOSI=%d D1(unused)=%d",
             LCD_WIDTH, LCD_HEIGHT, LCD_SPI_CLOCK_HZ / 1000000,
             PIN_LCD_CS, PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_D1);
    return ESP_OK;
}

esp_err_t nv3030b_lcd_fill(uint16_t rgb565)
{
    if (!s_lcd_mutex || xSemaphoreTake(s_lcd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, rgb565);
    xSemaphoreGive(s_lcd_mutex);
    return err;
}

esp_err_t nv3030b_lcd_show_test_pattern()
{
    if (!s_lcd_mutex || xSemaphoreTake(s_lcd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    constexpr uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
    const uint16_t bar_width = LCD_WIDTH / 4;
    esp_err_t err = ESP_OK;
    for (size_t i = 0; i < 4; ++i) {
        err = fill_rect(i * bar_width, 0, bar_width, LCD_HEIGHT, colors[i]);
        if (err != ESP_OK) {
            break;
        }
    }
    xSemaphoreGive(s_lcd_mutex);
    return err;
}

esp_err_t nv3030b_lcd_present(const uint16_t *pixels, size_t pixel_count)
{
    if (!s_lcd_mutex || xSemaphoreTake(s_lcd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = present_rect_locked(0, 0, LCD_WIDTH, LCD_HEIGHT, pixels, pixel_count);
    xSemaphoreGive(s_lcd_mutex);
    return err;
}

esp_err_t nv3030b_lcd_present_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                   const uint16_t *pixels, size_t pixel_count)
{
    if (!s_lcd_mutex || xSemaphoreTake(s_lcd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = present_rect_locked(x, y, width, height, pixels, pixel_count);
    xSemaphoreGive(s_lcd_mutex);
    return err;
}
