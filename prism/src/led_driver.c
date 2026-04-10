/**
 * WS281x LED strip driver using ESP-IDF's led_strip component (RMT backend).
 */

#include "led_driver.h"
#include "esp_log.h"
#include "led_strip.h"
#include <string.h>

static const char *TAG = "led_driver";

static led_strip_state_t s_strips[PRISM_STRIP_COUNT];
static int s_gpio_pins[PRISM_STRIP_COUNT];

// ========================= Init =============================================

void led_driver_init(const int gpio_pins[PRISM_STRIP_COUNT]) {
    for (int i = 0; i < PRISM_STRIP_COUNT; i++) {
        s_gpio_pins[i] = gpio_pins[i];
        s_strips[i].gpio = gpio_pins[i];
        s_strips[i].pixel_count = 0;
        s_strips[i].color_order = COLOR_ORDER_GRB;
        s_strips[i].configured = false;
        s_strips[i].strip_handle = NULL;
        memset(s_strips[i].pixel_data, 0, sizeof(s_strips[i].pixel_data));
    }
}

// ========================= Configure ========================================

bool led_driver_configure_strip(int strip, uint16_t pixel_count, color_order_t order) {
    if (strip < 0 || strip >= PRISM_STRIP_COUNT) {
        ESP_LOGE(TAG, "Invalid strip index: %d", strip);
        return false;
    }

    if (pixel_count == 0 || pixel_count > PRISM_MAX_PIXELS_PER_STRIP) {
        ESP_LOGE(TAG, "Invalid pixel count: %d", pixel_count);
        return false;
    }

    led_strip_state_t *s = &s_strips[strip];

    // Tear down existing strip if reconfiguring
    if (s->strip_handle != NULL) {
        led_strip_del(s->strip_handle);
        s->strip_handle = NULL;
    }

    // Configure the LED strip via RMT
    led_strip_config_t strip_config = {
        .strip_gpio_num = s_gpio_pins[strip],
        .max_leds = pixel_count,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz — standard for WS2812
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s->strip_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip %d: %s", strip, esp_err_to_name(err));
        s->configured = false;
        return false;
    }

    s->pixel_count = pixel_count;
    s->color_order = order;
    s->configured = true;
    memset(s->pixel_data, 0, pixel_count * 3);

    // Clear strip
    led_strip_clear(s->strip_handle);

    ESP_LOGI(TAG, "Strip %d configured: %d pixels, order %d, GPIO %d",
             strip, pixel_count, order, s_gpio_pins[strip]);
    return true;
}

// ========================= Update ===========================================

bool led_driver_update_strip(int strip, const uint8_t *rgb_data, size_t data_len) {
    if (strip < 0 || strip >= PRISM_STRIP_COUNT) {
        return false;
    }

    led_strip_state_t *s = &s_strips[strip];
    if (!s->configured || s->strip_handle == NULL) {
        return false;
    }

    size_t expected_len = (size_t)s->pixel_count * 3;
    if (data_len < expected_len) {
        return false;
    }

    // Copy RGB data into our buffer
    memcpy(s->pixel_data, rgb_data, expected_len);

    // Set each pixel on the strip handle.
    // Input from RoboRIO is always RGB. The led_strip component handles
    // WS2812 GRB wire encoding internally via the LED model setting.
    for (int i = 0; i < s->pixel_count; i++) {
        int offset = i * 3;
        led_strip_set_pixel(s->strip_handle, i,
                            rgb_data[offset],       // R
                            rgb_data[offset + 1],   // G
                            rgb_data[offset + 2]);   // B
    }

    return true;
}

// ========================= Refresh ==========================================

bool led_driver_refresh_strip(int strip) {
    if (strip < 0 || strip >= PRISM_STRIP_COUNT) {
        return false;
    }

    led_strip_state_t *s = &s_strips[strip];
    if (!s->configured || s->strip_handle == NULL) {
        return false;
    }

    esp_err_t err = led_strip_refresh(s->strip_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh strip %d: %s", strip, esp_err_to_name(err));
        return false;
    }

    return true;
}

void led_driver_refresh_all(void) {
    for (int i = 0; i < PRISM_STRIP_COUNT; i++) {
        if (s_strips[i].configured) {
            led_driver_refresh_strip(i);
        }
    }
}

// ========================= Accessors ========================================

const led_strip_state_t *led_driver_get_strip(int strip) {
    if (strip < 0 || strip >= PRISM_STRIP_COUNT) {
        return NULL;
    }
    return &s_strips[strip];
}
