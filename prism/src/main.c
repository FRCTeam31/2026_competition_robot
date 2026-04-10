/**
 * Prism main application — USB CDC serial rx/tx, protocol dispatch, LED driving.
 *
 * Architecture:
 *   1. Main task reads USB CDC serial data into a parse buffer
 *   2. Parse buffer is scanned for valid protocol frames
 *   3. Frames are dispatched to command handlers
 *   4. PIXEL_DATA_ALL frames update strip buffers and trigger RMT refresh
 *   5. Heartbeat responses and config ACKs are sent back over CDC tx
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"

#include "prism_app.h"
#include "protocol.h"
#include "led_driver.h"

static const char *TAG = "prism";

// ========================= State ============================================

static uint8_t s_parse_buf[PRISM_MAX_FRAME_SIZE];
static size_t s_parse_buf_len = 0;

// Tx buffer for response frames
static uint8_t s_tx_buf[32];

// ========================= Uptime ===========================================

static uint32_t get_uptime_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ========================= Serial I/O =======================================

static void serial_send(const uint8_t *data, size_t len) {
    size_t written = 0;
    while (written < len) {
        int ret = usb_serial_jtag_write_bytes(data + written, len - written, pdMS_TO_TICKS(10));
        if (ret < 0) {
            ESP_LOGE(TAG, "USB write error");
            break;
        }
        written += ret;
    }
}

// ========================= Command Handlers =================================

static void handle_configure(const uint8_t *payload, uint16_t len) {
    if (len < 4) {
        ESP_LOGW(TAG, "CONFIGURE payload too short: %d", len);
        return;
    }

    uint8_t strip = payload[0];
    uint16_t pixel_count = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
    color_order_t order = (color_order_t)payload[3];

    ESP_LOGI(TAG, "CONFIGURE strip=%d pixels=%d order=%d", strip, pixel_count, order);

    bool ok = led_driver_configure_strip(strip, pixel_count, order);

    // Send CONFIG_ACK
    size_t frame_len = prism_build_config_ack(s_tx_buf, strip,
                                               ok ? PRISM_STATUS_OK : PRISM_STATUS_ERROR);
    serial_send(s_tx_buf, frame_len);
}

static void handle_pixel_data(const uint8_t *payload, uint16_t len) {
    if (len < 3) { // At least strip index + 2 bytes for pixel count
        return;
    }

    uint8_t strip = payload[0];
    if (strip >= PRISM_STRIP_COUNT) {
        return;
    }

    const uint8_t *rgb_data = &payload[1];
    size_t rgb_len = len - 1;

    led_driver_update_strip(strip, rgb_data, rgb_len);
    led_driver_refresh_strip(strip);
}

static void handle_pixel_data_all(const uint8_t *payload, uint16_t len) {
    size_t offset = 0;

    for (int strip = 0; strip < PRISM_STRIP_COUNT; strip++) {
        // Need at least 2 bytes for pixel count
        if (offset + 2 > len) {
            ESP_LOGW(TAG, "PIXEL_DATA_ALL truncated at strip %d", strip);
            break;
        }

        uint16_t pixel_count = (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
        offset += 2;

        size_t rgb_len = (size_t)pixel_count * 3;
        if (offset + rgb_len > len) {
            ESP_LOGW(TAG, "PIXEL_DATA_ALL truncated RGB data for strip %d", strip);
            break;
        }

        led_driver_update_strip(strip, &payload[offset], rgb_len);
        offset += rgb_len;
    }

    // Refresh all strips together for synchronized display
    led_driver_refresh_all();
}

static void handle_heartbeat_req(void) {
    size_t frame_len = prism_build_heartbeat_rsp(s_tx_buf,
                                                  get_uptime_ms(),
                                                  PRISM_FIRMWARE_VERSION,
                                                  PRISM_STATUS_OK);
    serial_send(s_tx_buf, frame_len);
}

// ========================= Frame Dispatch ===================================

static void dispatch_frame(const prism_frame_t *frame) {
    switch (frame->command) {
        case PRISM_CMD_CONFIGURE:
            handle_configure(frame->payload, frame->payload_len);
            break;
        case PRISM_CMD_PIXEL_DATA:
            handle_pixel_data(frame->payload, frame->payload_len);
            break;
        case PRISM_CMD_PIXEL_DATA_ALL:
            handle_pixel_data_all(frame->payload, frame->payload_len);
            break;
        case PRISM_CMD_HEARTBEAT_REQ:
            handle_heartbeat_req();
            break;
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", frame->command);
            break;
    }
}

// ========================= Main Task ========================================

static void prism_main_task(void *arg) {
    ESP_LOGI(TAG, "Prism main task started");

    while (1) {
        // Read available data from USB serial into parse buffer
        int bytes_read = usb_serial_jtag_read_bytes(
            s_parse_buf + s_parse_buf_len,
            sizeof(s_parse_buf) - s_parse_buf_len,
            pdMS_TO_TICKS(1)
        );

        if (bytes_read > 0) {
            s_parse_buf_len += bytes_read;
        }

        // Try to parse and dispatch frames from the buffer
        while (s_parse_buf_len >= PRISM_FRAME_OVERHEAD) {
            prism_frame_t frame;
            size_t consumed = prism_parse_frame(s_parse_buf, s_parse_buf_len, &frame);

            if (consumed == 0) {
                // No complete frame yet — if the buffer is getting full and we
                // haven't found a sync, discard some data to make room
                if (s_parse_buf_len > PRISM_MAX_FRAME_SIZE / 2) {
                    // Try to find the next sync byte pair to realign
                    size_t discard = 1;
                    for (size_t i = 1; i < s_parse_buf_len - 1; i++) {
                        if (s_parse_buf[i] == PRISM_SYNC_BYTE_1 &&
                            s_parse_buf[i + 1] == PRISM_SYNC_BYTE_2) {
                            discard = i;
                            break;
                        }
                        discard = i + 1;
                    }
                    memmove(s_parse_buf, s_parse_buf + discard, s_parse_buf_len - discard);
                    s_parse_buf_len -= discard;
                }
                break;
            }

            // Dispatch the parsed frame
            dispatch_frame(&frame);

            // Remove consumed bytes from the parse buffer
            if (consumed < s_parse_buf_len) {
                memmove(s_parse_buf, s_parse_buf + consumed, s_parse_buf_len - consumed);
            }
            s_parse_buf_len -= consumed;
        }
    }
}

// ========================= Entry Point ======================================

void app_main(void) {
    ESP_LOGI(TAG, "Prism firmware v%d starting", PRISM_FIRMWARE_VERSION);

    // Initialize USB Serial JTAG driver (CDC)
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    ESP_LOGI(TAG, "USB Serial JTAG driver installed");

    // Initialize LED driver with GPIO assignments
    const int gpio_pins[PRISM_STRIP_COUNT] = {
        STRIP_0_GPIO,
        STRIP_1_GPIO,
        STRIP_2_GPIO,
        STRIP_3_GPIO,
    };
    led_driver_init(gpio_pins);
    ESP_LOGI(TAG, "LED driver initialized");

    // Start the main processing task on core 0
    xTaskCreatePinnedToCore(prism_main_task, "prism_main", 8192, NULL, 10, NULL, 0);
}
