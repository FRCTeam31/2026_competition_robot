/**
 * Prism Protocol — Frame format and constants.
 *
 * Matches the Java-side protocol defined in org.prime.prism.Protocol and
 * org.prime.prism.Prism.PrismMap exactly.
 *
 * Frame format (binary, little-endian):
 *   [0xAA][0x55][cmd:1][payloadLen:2 LE][payload...][checksum:1]
 *
 * Checksum is XOR of all payload bytes.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ========================= Frame Framing ====================================

#define PRISM_SYNC_BYTE_1 0xAA
#define PRISM_SYNC_BYTE_2 0x55
#define PRISM_FRAME_OVERHEAD 6 // 2 sync + 1 cmd + 2 len + 1 checksum

// ========================= Commands (RoboRIO -> Device) =====================

#define PRISM_CMD_CONFIGURE 0x01
#define PRISM_CMD_PIXEL_DATA 0x02
#define PRISM_CMD_PIXEL_DATA_ALL 0x03
#define PRISM_CMD_HEARTBEAT_REQ 0x04

// ========================= Commands (Device -> RoboRIO) =====================

#define PRISM_CMD_CONFIG_ACK 0x81
#define PRISM_CMD_HEARTBEAT_RSP 0x84

// ========================= Status Codes =====================================

#define PRISM_STATUS_OK 0x00
#define PRISM_STATUS_ERROR 0x01

// ========================= Limits ===========================================

#ifndef PRISM_STRIP_COUNT
#define PRISM_STRIP_COUNT 4
#endif

#ifndef PRISM_MAX_PIXELS_PER_STRIP
#define PRISM_MAX_PIXELS_PER_STRIP 144
#endif

// Maximum payload: 4 strips * (2 len bytes + 144 * 3 RGB bytes) = 1736
#define PRISM_MAX_PAYLOAD_SIZE (PRISM_STRIP_COUNT * (2 + PRISM_MAX_PIXELS_PER_STRIP * 3))
#define PRISM_MAX_FRAME_SIZE (PRISM_FRAME_OVERHEAD + PRISM_MAX_PAYLOAD_SIZE)

// ========================= Color Order ======================================

typedef enum
{
    COLOR_ORDER_RGB = 0x00,
    COLOR_ORDER_GRB = 0x01,
    COLOR_ORDER_RGBW = 0x02,
    COLOR_ORDER_GRBW = 0x03,
} color_order_t;

// ========================= Parsed Frame =====================================

typedef struct
{
    uint8_t command;
    uint16_t payload_len;
    const uint8_t *payload; // Points into the receive buffer — not owned
} prism_frame_t;

// ========================= Protocol Functions ===============================

/**
 * Compute XOR checksum over a payload buffer.
 */
uint8_t prism_checksum(const uint8_t *payload, size_t len);

/**
 * Try to parse a complete frame from the ring-style receive buffer.
 *
 * @param buf       Pointer to buffer data
 * @param buf_len   Number of valid bytes in buffer
 * @param out_frame Parsed frame output (payload pointer is into buf)
 * @return          Number of bytes consumed (0 if no valid frame found)
 */
size_t prism_parse_frame(const uint8_t *buf, size_t buf_len, prism_frame_t *out_frame);

/**
 * Build a CONFIG_ACK response frame.
 *
 * @param out       Output buffer (must be at least PRISM_FRAME_OVERHEAD + 2 bytes)
 * @param strip     Strip index being acknowledged
 * @param status    Status code (PRISM_STATUS_OK or PRISM_STATUS_ERROR)
 * @return          Total frame length written
 */
size_t prism_build_config_ack(uint8_t *out, uint8_t strip, uint8_t status);

/**
 * Build a HEARTBEAT_RSP response frame.
 *
 * @param out              Output buffer (must be at least PRISM_FRAME_OVERHEAD + 7 bytes)
 * @param uptime_ms        Device uptime in milliseconds
 * @param firmware_version Firmware version (uint16)
 * @param status           Device status code
 * @return                 Total frame length written
 */
size_t prism_build_heartbeat_rsp(uint8_t *out, uint32_t uptime_ms,
                                 uint16_t firmware_version, uint8_t status);
