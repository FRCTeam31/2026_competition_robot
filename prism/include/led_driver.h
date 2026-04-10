/**
 * WS281x LED strip driver using the ESP32 RMT peripheral.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"
#include "led_strip.h"

// ========================= Strip State ======================================

typedef struct {
    int gpio;
    uint16_t pixel_count;
    color_order_t color_order;
    bool configured;
    led_strip_handle_t strip_handle;
    uint8_t pixel_data[PRISM_MAX_PIXELS_PER_STRIP * 3]; // RGB buffer
} led_strip_state_t;

// ========================= API ==============================================

/**
 * Initialize the LED driver (call once at startup).
 * Sets all strips to unconfigured state.
 *
 * @param gpio_pins  Array of PRISM_STRIP_COUNT GPIO pin numbers
 */
void led_driver_init(const int gpio_pins[PRISM_STRIP_COUNT]);

/**
 * Configure a strip with the given pixel count and color order.
 * (Re)initializes the RMT channel for the strip.
 *
 * @param strip       Strip index (0 to PRISM_STRIP_COUNT-1)
 * @param pixel_count Number of pixels
 * @param order       Color order
 * @return            true on success
 */
bool led_driver_configure_strip(int strip, uint16_t pixel_count, color_order_t order);

/**
 * Update a strip's pixel data from an RGB byte array.
 * The data is expected in RGB order regardless of the strip's color_order —
 * reordering is handled internally.
 *
 * @param strip      Strip index
 * @param rgb_data   Pointer to pixel_count * 3 bytes of RGB data
 * @param data_len   Length of rgb_data in bytes
 * @return           true on success
 */
bool led_driver_update_strip(int strip, const uint8_t *rgb_data, size_t data_len);

/**
 * Refresh a strip — push buffered pixel data to the physical LEDs.
 *
 * @param strip  Strip index
 * @return       true on success
 */
bool led_driver_refresh_strip(int strip);

/**
 * Refresh all configured strips.
 */
void led_driver_refresh_all(void);

/**
 * Get the strip state for a given index.
 */
const led_strip_state_t *led_driver_get_strip(int strip);
