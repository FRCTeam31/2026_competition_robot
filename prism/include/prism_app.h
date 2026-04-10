/**
 * Prism main application — ties together USB CDC serial, protocol parsing,
 * and LED strip driving.
 */

#pragma once

#include <stdint.h>

#define PRISM_FIRMWARE_VERSION 0x0001

/**
 * GPIO pin assignments for LED strips on the XIAO ESP32-C3.
 */
#define STRIP_0_GPIO 2
#define STRIP_1_GPIO 3
#define STRIP_2_GPIO 4
#define STRIP_3_GPIO 5

/**
 * USB CDC receive buffer size.
 * Must hold at least one max-size PIXEL_DATA_ALL frame.
 */
#define RX_BUF_SIZE 2048
