package org.prime.prism;

import org.prime.prism.Prism.PrismMap;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Static utilities for building and parsing Prism serial protocol frames.
 *
 * <p>Frame format (binary, little-endian):
 * <pre>
 * [0xAA][0x55][cmd:1][payloadLen:2 LE][payload...][checksum:1]
 * </pre>
 *
 * Checksum is XOR of all payload bytes.
 */
public final class Protocol {

    private Protocol() {
    }

    // ========================= Response Types ===============================

    public sealed interface PrismResponse permits ConfigAck, HeartbeatResponse {
    }

    public record ConfigAck(int stripIndex, int status) implements PrismResponse {
    }

    public record HeartbeatResponse(long uptimeMs, int firmwareVersion, int status) implements PrismResponse {
    }

    // ========================= Frame Building ===============================

    /**
     * Builds a CONFIGURE frame for a single strip.
     *
     * @param strip      Strip index (0-3)
     * @param pixelCount Number of pixels on this strip
     * @param order      Color order for the strip's LEDs
     * @return The complete frame bytes ready for serial transmission
     */
    public static byte[] buildConfigureFrame(int strip, int pixelCount, Prism.ColorOrder order) {
        byte[] payload = new byte[4];
        payload[0] = (byte) strip;
        payload[1] = (byte) (pixelCount & 0xFF);
        payload[2] = (byte) ((pixelCount >> 8) & 0xFF);
        payload[3] = order.value;
        return wrapFrame(PrismMap.CMD_CONFIGURE, payload);
    }

    /**
     * Builds a PIXEL_DATA_ALL frame containing pixel data for all strips.
     *
     * <p>Payload layout: For each strip in order, a 2-byte LE pixel count
     * followed by (count * 3) RGB bytes.
     *
     * @param buffers Array of 4 AddressableLEDBuffers, one per strip
     * @return The complete frame bytes ready for serial transmission
     */
    public static byte[] buildPixelDataAllFrame(AddressableLEDBuffer[] buffers) {
        // Calculate total payload size: for each strip, 2 bytes for length + 3 bytes per pixel
        int payloadSize = 0;
        for (var buffer : buffers) {
            payloadSize += 2 + (buffer.getLength() * 3);
        }

        byte[] payload = new byte[payloadSize];
        int offset = 0;

        for (var buffer : buffers) {
            int len = buffer.getLength();
            // Pixel count as uint16 LE
            payload[offset++] = (byte) (len & 0xFF);
            payload[offset++] = (byte) ((len >> 8) & 0xFF);

            // RGB data for each pixel
            for (int i = 0; i < len; i++) {
                var color = buffer.getLED(i);
                payload[offset++] = (byte) (color.red * 255);
                payload[offset++] = (byte) (color.green * 255);
                payload[offset++] = (byte) (color.blue * 255);
            }
        }

        return wrapFrame(PrismMap.CMD_PIXEL_DATA_ALL, payload);
    }

    /**
     * Builds a HEARTBEAT_REQ frame (empty payload).
     *
     * @return The complete frame bytes
     */
    public static byte[] buildHeartbeatRequest() {
        return wrapFrame(PrismMap.CMD_HEARTBEAT_REQ, new byte[0]);
    }

    // ========================= Response Parsing =============================

    /**
     * Attempts to parse a response frame from the device.
     *
     * @param data Raw bytes received from the serial port
     * @return The parsed response, or null if the frame is invalid
     */
    public static PrismResponse parseResponse(byte[] data) {
        if (data == null || data.length < PrismMap.FRAME_OVERHEAD) {
            return null;
        }

        // Validate sync bytes
        if (data[0] != PrismMap.SYNC_BYTE_1 || data[1] != PrismMap.SYNC_BYTE_2) {
            return null;
        }

        byte cmd = data[2];
        int payloadLen = (data[3] & 0xFF) | ((data[4] & 0xFF) << 8);

        // Validate frame length
        if (data.length < PrismMap.FRAME_OVERHEAD + payloadLen) {
            return null;
        }

        // Extract payload
        byte[] payload = new byte[payloadLen];
        System.arraycopy(data, 5, payload, 0, payloadLen);

        // Validate checksum
        byte expectedChecksum = computeChecksum(payload);
        byte actualChecksum = data[5 + payloadLen];
        if (expectedChecksum != actualChecksum) {
            return null;
        }

        return switch (cmd) {
            case PrismMap.CMD_CONFIG_ACK -> parseConfigAck(payload);
            case PrismMap.CMD_HEARTBEAT_RSP -> parseHeartbeatResponse(payload);
            default -> null;
        };
    }

    /**
     * Searches for a valid response frame within a byte buffer that may contain
     * partial or multiple frames. Scans for sync bytes and attempts to parse.
     *
     * @param data Raw bytes from serial read
     * @param length Number of valid bytes in the data array
     * @return The parsed response, or null if no valid frame found
     */
    public static PrismResponse findAndParseResponse(byte[] data, int length) {
        for (int i = 0; i <= length - PrismMap.FRAME_OVERHEAD; i++) {
            if (data[i] == PrismMap.SYNC_BYTE_1 && (i + 1 < length) && data[i + 1] == PrismMap.SYNC_BYTE_2) {
                // Found potential frame start — try to parse from here
                int remaining = length - i;
                byte[] candidate = new byte[remaining];
                System.arraycopy(data, i, candidate, 0, remaining);
                PrismResponse response = parseResponse(candidate);
                if (response != null) {
                    return response;
                }
            }
        }
        return null;
    }

    // ========================= Checksum =====================================

    /**
     * Computes XOR checksum over all payload bytes.
     *
     * @param payload The payload bytes
     * @return XOR of all bytes
     */
    public static byte computeChecksum(byte[] payload) {
        byte checksum = 0;
        for (byte b : payload) {
            checksum ^= b;
        }
        return checksum;
    }

    // ========================= Internal Helpers =============================

    private static byte[] wrapFrame(byte command, byte[] payload) {
        int frameLen = PrismMap.FRAME_OVERHEAD + payload.length;
        byte[] frame = new byte[frameLen];

        frame[0] = PrismMap.SYNC_BYTE_1;
        frame[1] = PrismMap.SYNC_BYTE_2;
        frame[2] = command;
        frame[3] = (byte) (payload.length & 0xFF);
        frame[4] = (byte) ((payload.length >> 8) & 0xFF);

        System.arraycopy(payload, 0, frame, 5, payload.length);

        frame[frameLen - 1] = computeChecksum(payload);

        return frame;
    }

    private static ConfigAck parseConfigAck(byte[] payload) {
        if (payload.length < 2) {
            return null;
        }
        return new ConfigAck(payload[0] & 0xFF, payload[1] & 0xFF);
    }

    private static HeartbeatResponse parseHeartbeatResponse(byte[] payload) {
        if (payload.length < 7) {
            return null;
        }
        long uptimeMs = (payload[0] & 0xFFL)
                | ((payload[1] & 0xFFL) << 8)
                | ((payload[2] & 0xFFL) << 16)
                | ((payload[3] & 0xFFL) << 24);
        int firmwareVersion = (payload[4] & 0xFF) | ((payload[5] & 0xFF) << 8);
        int status = payload[6] & 0xFF;
        return new HeartbeatResponse(uptimeMs, firmwareVersion, status);
    }
}
