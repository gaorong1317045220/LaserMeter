#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Requested local-preview pipeline:
//   1x/2x: VGA JPEG 640x480 -> scale 320x240 -> rotate -> 240x320.
//   1x discards 18 rows at both ends; 2x performs an explicit 120x142 centre
//   crop and scales it to the 240x284 LCD in device_ui_lvgl.cpp.
//
// Keeping the decoder output identical for both zoom modes avoids a large
// full-VGA temporary buffer and makes zoom switching non-blocking for the UI.
#define CAMERA_JPEG_SCALED_WIDTH          320u
#define CAMERA_JPEG_SCALED_HEIGHT         240u
#define CAMERA_JPEG_ROTATED_WIDTH         240u
#define CAMERA_JPEG_ROTATED_HEIGHT        320u
#define CAMERA_JPEG_DISPLAY_WIDTH         240u
#define CAMERA_JPEG_DISPLAY_HEIGHT        284u
#define CAMERA_JPEG_DISPLAY_CROP_ROWS     18u
#define CAMERA_JPEG_OUTPUT_PIXEL_COUNT    \
    (CAMERA_JPEG_ROTATED_WIDTH * CAMERA_JPEG_ROTATED_HEIGHT)
#define CAMERA_JPEG_OUTPUT_BYTES          \
    (CAMERA_JPEG_OUTPUT_PIXEL_COUNT * sizeof(uint16_t))
#define CAMERA_JPEG_DISPLAY_OFFSET_PIXELS \
    (CAMERA_JPEG_DISPLAY_CROP_ROWS * CAMERA_JPEG_ROTATED_WIDTH)

typedef enum {
    CAMERA_JPEG_ROTATE_CLOCKWISE_90 = 0,
    CAMERA_JPEG_ROTATE_CLOCKWISE_270,
} camera_jpeg_rotation_t;

typedef struct {
    camera_jpeg_rotation_t rotation;
    bool center_crop_2x;
} camera_jpeg_decoder_config_t;

#define CAMERA_JPEG_DECODER_DEFAULT_CONFIG() \
    { CAMERA_JPEG_ROTATE_CLOCKWISE_270, false }

typedef struct camera_jpeg_decoder camera_jpeg_decoder_t;

typedef struct {
    // ESP_NEW_JPEG's parsed dimensions, retained for diagnostics.
    uint16_t header_width;
    uint16_t header_height;

    // Full decoder output. Bytes are already RGB565 big-endian (LCD wire
    // order); the uint16_t numeric value is byte-swapped on ESP32-S3.
    uint16_t *output_pixels;
    size_t output_pixel_count;

    // Contiguous, zero-copy 240x284 centre crop ready for the portrait LCD.
    uint16_t *display_pixels;
    uint16_t display_width;
    uint16_t display_height;
    uint16_t display_stride_pixels;

    int codec_error;
    int64_t parse_us;
    int64_t process_us;
    int64_t total_us;
} camera_jpeg_decoded_frame_t;

// Opens one persistent decoder handle.  Keep it for the lifetime of the
// camera producer task; repeatedly opening/closing it wastes time and
// fragments scarce internal RAM.  One instance must only be used by one task.
esp_err_t camera_jpeg_decoder_create(const camera_jpeg_decoder_config_t *config,
                                     camera_jpeg_decoder_t **out_decoder);

void camera_jpeg_decoder_destroy(camera_jpeg_decoder_t *decoder);

// Allocates the required 153600-byte, 16-byte-aligned output buffer strictly
// in PSRAM.  Two buffers may be allocated for producer/display pipelining.
uint16_t *camera_jpeg_decoder_alloc_output(void);
void camera_jpeg_decoder_free_output(uint16_t *pixels);

// Validates the alignment, capacity and PSRAM placement required by the S3
// SIMD decoder.  Passing a normal heap buffer is intentionally rejected so
// Wi-Fi's internal-RAM headroom cannot be consumed accidentally.
bool camera_jpeg_decoder_output_is_valid(const uint16_t *pixels,
                                         size_t pixel_capacity);

// Decodes one baseline VGA JPEG into the caller-owned output buffer.  JPEG
// capture ownership/mutex handling remains with the caller.  Return the camera
// frame buffer only after this function finishes because the decoder reads it
// directly and never copies the compressed frame.
esp_err_t camera_jpeg_decoder_decode(camera_jpeg_decoder_t *decoder,
                                     const uint8_t *jpeg,
                                     size_t jpeg_size,
                                     uint16_t *output_pixels,
                                     size_t output_pixel_capacity,
                                     camera_jpeg_decoded_frame_t *out_frame);

#ifdef __cplusplus
}
#endif
