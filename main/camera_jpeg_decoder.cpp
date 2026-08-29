#include "camera_jpeg_decoder.h"

#include <climits>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_jpeg_dec.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"

namespace {

constexpr uint32_t kDecoderMagic = 0x4A504744u;  // "JPGD"
constexpr size_t kOutputAlignment = 16u;

esp_err_t codec_error_to_esp(jpeg_error_t err)
{
    switch (err) {
    case JPEG_ERR_OK: return ESP_OK;
    case JPEG_ERR_NO_MEM: return ESP_ERR_NO_MEM;
    case JPEG_ERR_INVALID_PARAM: return ESP_ERR_INVALID_ARG;
    case JPEG_ERR_NO_MORE_DATA: return ESP_ERR_INVALID_SIZE;
    case JPEG_ERR_UNSUPPORT_FMT:
    case JPEG_ERR_UNSUPPORT_STD: return ESP_ERR_NOT_SUPPORTED;
    case JPEG_ERR_BAD_DATA: return ESP_ERR_INVALID_RESPONSE;
    case JPEG_ERR_FAIL:
    default: return ESP_FAIL;
    }
}

void initialise_result(camera_jpeg_decoded_frame_t *frame,
                       uint16_t *output_pixels)
{
    if (!frame) return;
    std::memset(frame, 0, sizeof(*frame));
    frame->output_pixels = output_pixels;
    frame->output_pixel_count = CAMERA_JPEG_OUTPUT_PIXEL_COUNT;
    frame->display_pixels = output_pixels
                                ? output_pixels + CAMERA_JPEG_DISPLAY_OFFSET_PIXELS
                                : nullptr;
    frame->display_width = CAMERA_JPEG_DISPLAY_WIDTH;
    frame->display_height = CAMERA_JPEG_DISPLAY_HEIGHT;
    frame->display_stride_pixels = CAMERA_JPEG_ROTATED_WIDTH;
    frame->codec_error = JPEG_ERR_OK;
}

}  // namespace

struct camera_jpeg_decoder {
    jpeg_dec_handle_t handle;
    uint32_t magic;
};

extern "C" esp_err_t camera_jpeg_decoder_create(
    const camera_jpeg_decoder_config_t *config,
    camera_jpeg_decoder_t **out_decoder)
{
    if (!out_decoder) return ESP_ERR_INVALID_ARG;
    *out_decoder = nullptr;

    camera_jpeg_decoder_config_t requested = CAMERA_JPEG_DECODER_DEFAULT_CONFIG();
    if (config) requested = *config;
    if (requested.rotation != CAMERA_JPEG_ROTATE_CLOCKWISE_90 &&
        requested.rotation != CAMERA_JPEG_ROTATE_CLOCKWISE_270) {
        return ESP_ERR_INVALID_ARG;
    }

    auto *decoder = static_cast<camera_jpeg_decoder_t *>(
        heap_caps_calloc(1, sizeof(camera_jpeg_decoder_t),
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!decoder) return ESP_ERR_NO_MEM;

    jpeg_dec_config_t jpeg_config = DEFAULT_JPEG_DEC_CONFIG();
    // Big-endian output can be sent to the LCD without a full-frame byte-swap.
    jpeg_config.output_type = JPEG_PIXEL_FORMAT_RGB565_BE;
    // Decode the same deterministic 320x240 image for both zoom modes. The
    // 2x centre crop is performed by copy_camera_crop_to_lvgl(), where the
    // crop origin is explicit and the output is scaled to the LCD. Avoiding
    // a full VGA decode keeps frame latency and PSRAM pressure unchanged when
    // the user toggles zoom.
    jpeg_config.scale.width = CAMERA_JPEG_SCALED_WIDTH;
    jpeg_config.scale.height = CAMERA_JPEG_SCALED_HEIGHT;
    jpeg_config.clipper.width = 0;
    jpeg_config.clipper.height = 0;
    jpeg_config.rotate = requested.rotation == CAMERA_JPEG_ROTATE_CLOCKWISE_90
                             ? JPEG_ROTATE_90D
                             : JPEG_ROTATE_270D;
    jpeg_config.block_enable = false;

    const jpeg_error_t codec_err = jpeg_dec_open(&jpeg_config, &decoder->handle);
    if (codec_err != JPEG_ERR_OK) {
        heap_caps_free(decoder);
        return codec_error_to_esp(codec_err);
    }

    decoder->magic = kDecoderMagic;
    *out_decoder = decoder;
    return ESP_OK;
}

extern "C" void camera_jpeg_decoder_destroy(camera_jpeg_decoder_t *decoder)
{
    if (!decoder) return;
    if (decoder->magic == kDecoderMagic && decoder->handle) {
        jpeg_dec_close(decoder->handle);
    }
    decoder->handle = nullptr;
    decoder->magic = 0;
    heap_caps_free(decoder);
}

extern "C" uint16_t *camera_jpeg_decoder_alloc_output(void)
{
    return static_cast<uint16_t *>(heap_caps_aligned_calloc(
        kOutputAlignment, 1, CAMERA_JPEG_OUTPUT_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

extern "C" void camera_jpeg_decoder_free_output(uint16_t *pixels)
{
    heap_caps_free(pixels);
}

extern "C" bool camera_jpeg_decoder_output_is_valid(const uint16_t *pixels,
                                                       size_t pixel_capacity)
{
    return pixels != nullptr &&
           pixel_capacity >= CAMERA_JPEG_OUTPUT_PIXEL_COUNT &&
           (reinterpret_cast<uintptr_t>(pixels) & (kOutputAlignment - 1u)) == 0u &&
           esp_ptr_external_ram(pixels);
}

extern "C" esp_err_t camera_jpeg_decoder_decode(
    camera_jpeg_decoder_t *decoder,
    const uint8_t *jpeg,
    size_t jpeg_size,
    uint16_t *output_pixels,
    size_t output_pixel_capacity,
    camera_jpeg_decoded_frame_t *out_frame)
{
    initialise_result(out_frame, output_pixels);
    const int64_t total_begin = esp_timer_get_time();

    if (!decoder || decoder->magic != kDecoderMagic || !decoder->handle ||
        !jpeg || jpeg_size < 4u || jpeg_size > static_cast<size_t>(INT_MAX) ||
        jpeg[0] != 0xFFu || jpeg[1] != 0xD8u ||
        !camera_jpeg_decoder_output_is_valid(output_pixels,
                                             output_pixel_capacity)) {
        if (out_frame) {
            out_frame->codec_error = JPEG_ERR_INVALID_PARAM;
            out_frame->total_us = esp_timer_get_time() - total_begin;
        }
        return ESP_ERR_INVALID_ARG;
    }

    jpeg_dec_io_t io = {};
    jpeg_dec_header_info_t header = {};
    io.inbuf = const_cast<uint8_t *>(jpeg);
    io.inbuf_len = static_cast<int>(jpeg_size);

    const int64_t parse_begin = esp_timer_get_time();
    jpeg_error_t codec_err = jpeg_dec_parse_header(decoder->handle, &io, &header);
    const int64_t process_begin = esp_timer_get_time();
    if (out_frame) {
        out_frame->header_width = header.width;
        out_frame->header_height = header.height;
        out_frame->parse_us = process_begin - parse_begin;
        out_frame->codec_error = codec_err;
    }
    if (codec_err != JPEG_ERR_OK) {
        if (out_frame) out_frame->total_us = process_begin - total_begin;
        return codec_error_to_esp(codec_err);
    }

    const size_t expected_output_bytes = CAMERA_JPEG_OUTPUT_BYTES;
    int required_output_bytes = 0;
    codec_err = jpeg_dec_get_outbuf_len(decoder->handle, &required_output_bytes);
    if (codec_err != JPEG_ERR_OK) {
        if (out_frame) {
            out_frame->codec_error = codec_err;
            out_frame->total_us = esp_timer_get_time() - total_begin;
        }
        return codec_error_to_esp(codec_err);
    }
    if (required_output_bytes != static_cast<int>(expected_output_bytes)) {
        if (out_frame) {
            out_frame->codec_error = JPEG_ERR_INVALID_PARAM;
            out_frame->total_us = esp_timer_get_time() - total_begin;
        }
        return ESP_ERR_INVALID_SIZE;
    }

    io.outbuf = reinterpret_cast<uint8_t *>(output_pixels);
    const int64_t decode_begin = esp_timer_get_time();
    codec_err = jpeg_dec_process(decoder->handle, &io);
    const int64_t decode_end = esp_timer_get_time();
    if (out_frame) {
        out_frame->codec_error = codec_err;
        out_frame->process_us = decode_end - decode_begin;
        out_frame->total_us = decode_end - total_begin;
    }
    if (codec_err != JPEG_ERR_OK) return codec_error_to_esp(codec_err);
    if (io.out_size != static_cast<int>(expected_output_bytes)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}
