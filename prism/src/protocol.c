/**
 * Prism Protocol implementation — frame parsing and response building.
 */

#include "protocol.h"
#include <string.h>

// ========================= Checksum =========================================

uint8_t prism_checksum(const uint8_t *payload, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }
    return checksum;
}

// ========================= Frame Parsing ====================================

size_t prism_parse_frame(const uint8_t *buf, size_t buf_len, prism_frame_t *out_frame) {
    // Scan for sync bytes
    for (size_t i = 0; i <= buf_len - PRISM_FRAME_OVERHEAD; i++) {
        if (buf[i] != PRISM_SYNC_BYTE_1) {
            continue;
        }
        if (buf[i + 1] != PRISM_SYNC_BYTE_2) {
            continue;
        }

        // Found sync — read header
        uint8_t cmd = buf[i + 2];
        uint16_t payload_len = (uint16_t)buf[i + 3] | ((uint16_t)buf[i + 4] << 8);

        // Check if we have the full frame
        size_t frame_len = PRISM_FRAME_OVERHEAD + payload_len;
        if (i + frame_len > buf_len) {
            // Incomplete frame — wait for more data
            return 0;
        }

        // Validate payload size
        if (payload_len > PRISM_MAX_PAYLOAD_SIZE) {
            // Skip this sync pair and keep scanning
            continue;
        }

        // Validate checksum
        const uint8_t *payload = &buf[i + 5];
        uint8_t expected_checksum = prism_checksum(payload, payload_len);
        uint8_t actual_checksum = buf[i + 5 + payload_len];

        if (expected_checksum != actual_checksum) {
            // Bad checksum — skip this sync pair
            continue;
        }

        // Valid frame
        out_frame->command = cmd;
        out_frame->payload_len = payload_len;
        out_frame->payload = payload;

        return i + frame_len; // Total bytes consumed (including any junk before sync)
    }

    return 0; // No valid frame found
}

// ========================= Response Building ================================

static size_t build_frame(uint8_t *out, uint8_t cmd, const uint8_t *payload, size_t payload_len) {
    out[0] = PRISM_SYNC_BYTE_1;
    out[1] = PRISM_SYNC_BYTE_2;
    out[2] = cmd;
    out[3] = (uint8_t)(payload_len & 0xFF);
    out[4] = (uint8_t)((payload_len >> 8) & 0xFF);

    if (payload_len > 0) {
        memcpy(&out[5], payload, payload_len);
    }

    out[5 + payload_len] = prism_checksum(payload, payload_len);

    return PRISM_FRAME_OVERHEAD + payload_len;
}

size_t prism_build_config_ack(uint8_t *out, uint8_t strip, uint8_t status) {
    uint8_t payload[2] = { strip, status };
    return build_frame(out, PRISM_CMD_CONFIG_ACK, payload, sizeof(payload));
}

size_t prism_build_heartbeat_rsp(uint8_t *out, uint32_t uptime_ms,
                                  uint16_t firmware_version, uint8_t status) {
    uint8_t payload[7];
    // uptime_ms as uint32 LE
    payload[0] = (uint8_t)(uptime_ms & 0xFF);
    payload[1] = (uint8_t)((uptime_ms >> 8) & 0xFF);
    payload[2] = (uint8_t)((uptime_ms >> 16) & 0xFF);
    payload[3] = (uint8_t)((uptime_ms >> 24) & 0xFF);
    // firmware_version as uint16 LE
    payload[4] = (uint8_t)(firmware_version & 0xFF);
    payload[5] = (uint8_t)((firmware_version >> 8) & 0xFF);
    // status
    payload[6] = status;

    return build_frame(out, PRISM_CMD_HEARTBEAT_RSP, payload, sizeof(payload));
}
