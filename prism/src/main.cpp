/**
 * Prism — USB Serial LED Controller Firmware (Arduino / XIAO ESP32-S3)
 *
 * Receives binary frames over USB CDC serial, drives up to 4 WS2812 strips
 * via Adafruit NeoPixel (RMT backend on ESP32).
 *
 * Protocol frame format (little-endian):
 *   [0xAA][0x55][cmd:1][payloadLen:2 LE][payload...][checksum:1]
 *   Checksum = XOR of all payload bytes
 *
 * Commands (host -> device):
 *   0x01 CONFIGURE      — payload: [strip:1][pixelCount:2 LE][colorOrder:1]
 *   0x02 PIXEL_DATA     — payload: [strip:1][RGB data...]
 *   0x03 PIXEL_DATA_ALL — payload: for each strip { [pixelCount:2 LE][RGB data...] }
 *   0x04 HEARTBEAT_REQ  — payload: (empty)
 *
 * Commands (device -> host):
 *   0x81 CONFIG_ACK     — payload: [strip:1][status:1]
 *   0x84 HEARTBEAT_RSP  — payload: [uptimeMs:4 LE][fwVersion:2 LE][status:1]
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ========================= Configuration ====================================

#define FIRMWARE_VERSION 0x0002

#ifndef PRISM_STRIP_COUNT
#define PRISM_STRIP_COUNT 4
#endif

#ifndef PRISM_MAX_PIXELS_PER_STRIP
#define PRISM_MAX_PIXELS_PER_STRIP 144
#endif

// GPIO pin assignments for XIAO ESP32-S3
static const uint8_t STRIP_PINS[PRISM_STRIP_COUNT] = { 1, 2, 4, 5 };

// ========================= Protocol Constants ===============================

#define SYNC1 0xAA
#define SYNC2 0x55

#define CMD_CONFIGURE      0x01
#define CMD_PIXEL_DATA     0x02
#define CMD_PIXEL_DATA_ALL 0x03
#define CMD_HEARTBEAT_REQ  0x04
#define CMD_CONFIG_ACK     0x81
#define CMD_HEARTBEAT_RSP  0x84

#define STATUS_OK    0x00
#define STATUS_ERROR 0x01

#define FRAME_OVERHEAD 6  // 2 sync + 1 cmd + 2 len + 1 checksum
#define MAX_PAYLOAD    (PRISM_STRIP_COUNT * (2 + PRISM_MAX_PIXELS_PER_STRIP * 3))
#define MAX_FRAME_SIZE (FRAME_OVERHEAD + MAX_PAYLOAD)

// ========================= Strip State ======================================

static Adafruit_NeoPixel* strips[PRISM_STRIP_COUNT] = { nullptr };
static uint16_t stripPixelCount[PRISM_STRIP_COUNT] = { 0 };
static bool stripConfigured[PRISM_STRIP_COUNT] = { false };

// ========================= Parse Buffer =====================================

static uint8_t parseBuf[MAX_FRAME_SIZE];
static size_t parseBufLen = 0;

// ========================= Protocol Helpers =================================

static uint8_t xorChecksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

static void sendFrame(uint8_t cmd, const uint8_t* payload, size_t payloadLen) {
    uint8_t header[5] = {
        SYNC1, SYNC2, cmd,
        (uint8_t)(payloadLen & 0xFF),
        (uint8_t)((payloadLen >> 8) & 0xFF)
    };
    Serial.write(header, 5);
    if (payloadLen > 0) {
        Serial.write(payload, payloadLen);
    }
    uint8_t cs = xorChecksum(payload, payloadLen);
    Serial.write(&cs, 1);
    Serial.flush();
}

// ========================= Command Handlers =================================

static void handleConfigure(const uint8_t* payload, uint16_t len) {
    if (len < 4) return;

    uint8_t strip = payload[0];
    uint16_t pixelCount = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
    // payload[3] = color order (ignored — NeoPixel handles GRB natively)

    if (strip >= PRISM_STRIP_COUNT || pixelCount == 0 || pixelCount > PRISM_MAX_PIXELS_PER_STRIP) {
        uint8_t ack[2] = { strip, STATUS_ERROR };
        sendFrame(CMD_CONFIG_ACK, ack, 2);
        return;
    }

    // Recreate strip object if needed
    if (strips[strip] != nullptr) {
        if (stripPixelCount[strip] == pixelCount) {
            // Same size — just clear and reuse
            strips[strip]->clear();
            strips[strip]->show();
            stripConfigured[strip] = true;
            uint8_t ack[2] = { strip, STATUS_OK };
            sendFrame(CMD_CONFIG_ACK, ack, 2);
            return;
        }
        // Different size — must recreate
        delete strips[strip];
        strips[strip] = nullptr;
    }

    strips[strip] = new Adafruit_NeoPixel(pixelCount, STRIP_PINS[strip], NEO_GRB + NEO_KHZ800);
    strips[strip]->begin();
    strips[strip]->clear();
    strips[strip]->show();

    stripPixelCount[strip] = pixelCount;
    stripConfigured[strip] = true;

    uint8_t ack[2] = { strip, STATUS_OK };
    sendFrame(CMD_CONFIG_ACK, ack, 2);
}

static void handlePixelData(const uint8_t* payload, uint16_t len) {
    if (len < 4) return;

    uint8_t strip = payload[0];
    if (strip >= PRISM_STRIP_COUNT || !stripConfigured[strip]) return;

    const uint8_t* rgb = &payload[1];
    size_t rgbLen = len - 1;
    uint16_t count = stripPixelCount[strip];

    if (rgbLen < (size_t)count * 3) return;

    for (uint16_t i = 0; i < count; i++) {
        strips[strip]->setPixelColor(i, rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
    }
    strips[strip]->show();
}

static void handlePixelDataAll(const uint8_t* payload, uint16_t len) {
    size_t offset = 0;

    for (int s = 0; s < PRISM_STRIP_COUNT; s++) {
        if (offset + 2 > len) break;

        uint16_t pixelCount = (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
        offset += 2;

        size_t rgbLen = (size_t)pixelCount * 3;
        if (offset + rgbLen > len) break;

        if (s < PRISM_STRIP_COUNT && stripConfigured[s] && strips[s] != nullptr) {
            uint16_t count = min(pixelCount, stripPixelCount[s]);
            for (uint16_t i = 0; i < count; i++) {
                const uint8_t* p = &payload[offset + i * 3];
                strips[s]->setPixelColor(i, p[0], p[1], p[2]);
            }
        }

        offset += rgbLen;
    }

    // Refresh all strips
    for (int s = 0; s < PRISM_STRIP_COUNT; s++) {
        if (stripConfigured[s] && strips[s] != nullptr) {
            strips[s]->show();
        }
    }
}

static void handleHeartbeatReq() {
    uint32_t uptime = millis();
    uint8_t payload[7] = {
        (uint8_t)(uptime & 0xFF),
        (uint8_t)((uptime >> 8) & 0xFF),
        (uint8_t)((uptime >> 16) & 0xFF),
        (uint8_t)((uptime >> 24) & 0xFF),
        (uint8_t)(FIRMWARE_VERSION & 0xFF),
        (uint8_t)((FIRMWARE_VERSION >> 8) & 0xFF),
        STATUS_OK
    };
    sendFrame(CMD_HEARTBEAT_RSP, payload, 7);
}

// ========================= Frame Parsing ====================================

static void processFrame(uint8_t cmd, const uint8_t* payload, uint16_t payloadLen) {
    switch (cmd) {
        case CMD_CONFIGURE:      handleConfigure(payload, payloadLen); break;
        case CMD_PIXEL_DATA:     handlePixelData(payload, payloadLen); break;
        case CMD_PIXEL_DATA_ALL: handlePixelDataAll(payload, payloadLen); break;
        case CMD_HEARTBEAT_REQ:  handleHeartbeatReq(); break;
    }
}

static void parseAndDispatch() {
    while (parseBufLen >= FRAME_OVERHEAD) {
        // Scan for sync bytes
        size_t syncPos = 0;
        bool found = false;
        for (size_t i = 0; i <= parseBufLen - FRAME_OVERHEAD; i++) {
            if (parseBuf[i] == SYNC1 && parseBuf[i + 1] == SYNC2) {
                syncPos = i;
                found = true;
                break;
            }
        }

        if (!found) {
            // No sync found — keep last byte (could be start of SYNC1)
            parseBuf[0] = parseBuf[parseBufLen - 1];
            parseBufLen = 1;
            return;
        }

        // Discard bytes before sync
        if (syncPos > 0) {
            memmove(parseBuf, parseBuf + syncPos, parseBufLen - syncPos);
            parseBufLen -= syncPos;
        }

        // Parse header
        uint8_t cmd = parseBuf[2];
        uint16_t payloadLen = (uint16_t)parseBuf[3] | ((uint16_t)parseBuf[4] << 8);

        // Validate payload size
        if (payloadLen > MAX_PAYLOAD) {
            // Bad frame — skip past sync and try again
            memmove(parseBuf, parseBuf + 2, parseBufLen - 2);
            parseBufLen -= 2;
            continue;
        }

        size_t frameLen = FRAME_OVERHEAD + payloadLen;

        // Wait for complete frame
        if (parseBufLen < frameLen) return;

        // Validate checksum
        const uint8_t* payload = &parseBuf[5];
        uint8_t expected = xorChecksum(payload, payloadLen);
        uint8_t actual = parseBuf[5 + payloadLen];

        if (expected == actual) {
            processFrame(cmd, payload, payloadLen);
        }

        // Consume frame
        if (frameLen < parseBufLen) {
            memmove(parseBuf, parseBuf + frameLen, parseBufLen - frameLen);
        }
        parseBufLen -= frameLen;
    }
}

// ========================= Arduino Entry Points =============================

void setup() {
    Serial.begin(115200);  // Baud rate is ignored for USB CDC, but required by API

    // Brief delay to let USB enumerate before we start processing
    delay(100);
}

void loop() {
    // Read available serial data into parse buffer
    int avail = Serial.available();
    if (avail > 0) {
        size_t space = sizeof(parseBuf) - parseBufLen;
        if (space > 0) {
            size_t toRead = min((size_t)avail, space);
            Serial.readBytes(parseBuf + parseBufLen, toRead);
            parseBufLen += toRead;
        }
    }

    parseAndDispatch();
}
