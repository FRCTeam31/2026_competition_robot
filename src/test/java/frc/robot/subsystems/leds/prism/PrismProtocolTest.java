package frc.robot.subsystems.leds.prism;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Unit tests for {@link PrismProtocol} — frame building, checksum, and response parsing.
 */
class PrismProtocolTest {

    // ========================= Frame Structure Tests ========================

    @Test
    void testWrapFrame_hasSyncBytes() {
        byte[] frame = PrismProtocol.buildHeartbeatRequest();
        assertEquals((byte) 0xAA, frame[0], "First sync byte");
        assertEquals((byte) 0x55, frame[1], "Second sync byte");
    }

    @Test
    void testWrapFrame_commandByte() {
        byte[] frame = PrismProtocol.buildHeartbeatRequest();
        assertEquals(PrismMap.CMD_HEARTBEAT_REQ, frame[2], "Command byte");
    }

    @Test
    void testHeartbeatRequest_emptyPayload() {
        byte[] frame = PrismProtocol.buildHeartbeatRequest();
        // 6 bytes overhead + 0 payload
        assertEquals(PrismMap.FRAME_OVERHEAD, frame.length);
        // Payload length should be 0
        assertEquals(0, frame[3] & 0xFF);
        assertEquals(0, frame[4] & 0xFF);
        // Checksum of empty payload is 0
        assertEquals(0, frame[5]);
    }

    // ========================= Configure Frame Tests ========================

    @Test
    void testBuildConfigureFrame_structure() {
        byte[] frame = PrismProtocol.buildConfigureFrame(2, 60, PrismMap.ColorOrder.GRB);

        assertEquals((byte) 0xAA, frame[0]);
        assertEquals((byte) 0x55, frame[1]);
        assertEquals(PrismMap.CMD_CONFIGURE, frame[2]);

        // Payload length = 4
        assertEquals(4, frame[3] & 0xFF);
        assertEquals(0, frame[4] & 0xFF);

        // Payload: strip index, pixel count LE, color order
        assertEquals(2, frame[5] & 0xFF); // strip index
        assertEquals(60, frame[6] & 0xFF); // pixel count low byte
        assertEquals(0, frame[7] & 0xFF); // pixel count high byte
        assertEquals(PrismMap.ColorOrder.GRB.value, frame[8]); // color order
    }

    @Test
    void testBuildConfigureFrame_largePixelCount() {
        byte[] frame = PrismProtocol.buildConfigureFrame(0, 300, PrismMap.ColorOrder.RGB);

        // 300 = 0x012C → low byte 0x2C, high byte 0x01
        assertEquals(0x2C, frame[6] & 0xFF);
        assertEquals(0x01, frame[7] & 0xFF);
    }

    // ========================= Pixel Data All Frame Tests ====================

    @Test
    void testBuildPixelDataAllFrame_singlePixelStrips() {
        AddressableLEDBuffer[] buffers = new AddressableLEDBuffer[4];
        for (int i = 0; i < 4; i++) {
            buffers[i] = new AddressableLEDBuffer(1);
            buffers[i].setLED(0, new Color(1.0, 0.0, 0.0)); // Red
        }

        byte[] frame = PrismProtocol.buildPixelDataAllFrame(buffers);

        assertEquals((byte) 0xAA, frame[0]);
        assertEquals((byte) 0x55, frame[1]);
        assertEquals(PrismMap.CMD_PIXEL_DATA_ALL, frame[2]);

        // Payload: 4 strips * (2 bytes length + 3 bytes RGB) = 20 bytes
        int expectedPayloadLen = 4 * (2 + 3);
        assertEquals(expectedPayloadLen, (frame[3] & 0xFF) | ((frame[4] & 0xFF) << 8));
        assertEquals(PrismMap.FRAME_OVERHEAD + expectedPayloadLen, frame.length);
    }

    @Test
    void testBuildPixelDataAllFrame_pixelDataCorrect() {
        AddressableLEDBuffer[] buffers = new AddressableLEDBuffer[4];
        for (int i = 0; i < 4; i++) {
            buffers[i] = new AddressableLEDBuffer(1);
        }
        // Set strip 0 to pure green
        buffers[0].setLED(0, new Color(0.0, 1.0, 0.0));

        byte[] frame = PrismProtocol.buildPixelDataAllFrame(buffers);

        // Strip 0: starts at payload offset 0 in frame[5..]
        // [pixelCount:2][R][G][B]
        int payloadStart = 5;
        assertEquals(1, frame[payloadStart] & 0xFF); // pixel count low
        assertEquals(0, frame[payloadStart + 1] & 0xFF); // pixel count high
        assertEquals(0, frame[payloadStart + 2] & 0xFF); // R = 0
        assertEquals((byte) 255, frame[payloadStart + 3]); // G = 255
        assertEquals(0, frame[payloadStart + 4] & 0xFF); // B = 0
    }

    @Test
    void testBuildPixelDataAllFrame_multiplePixels() {
        AddressableLEDBuffer[] buffers = new AddressableLEDBuffer[4];
        buffers[0] = new AddressableLEDBuffer(3); // 3 pixels
        buffers[1] = new AddressableLEDBuffer(2); // 2 pixels
        buffers[2] = new AddressableLEDBuffer(1); // 1 pixel
        buffers[3] = new AddressableLEDBuffer(0); // 0 pixels

        byte[] frame = PrismProtocol.buildPixelDataAllFrame(buffers);

        // Payload: (2+9) + (2+6) + (2+3) + (2+0) = 26 bytes
        int expectedPayload = 26;
        assertEquals(expectedPayload, (frame[3] & 0xFF) | ((frame[4] & 0xFF) << 8));
    }

    // ========================= Checksum Tests ================================

    @Test
    void testComputeChecksum_empty() {
        assertEquals(0, PrismProtocol.computeChecksum(new byte[0]));
    }

    @Test
    void testComputeChecksum_singleByte() {
        assertEquals((byte) 0x42, PrismProtocol.computeChecksum(new byte[] { 0x42 }));
    }

    @Test
    void testComputeChecksum_xorProperty() {
        // XOR of identical bytes = 0
        assertEquals(0, PrismProtocol.computeChecksum(new byte[] { 0x55, 0x55 }));
        // XOR is order-independent and associative
        byte[] data = { 0x01, 0x02, 0x03 };
        assertEquals((byte) (0x01 ^ 0x02 ^ 0x03), PrismProtocol.computeChecksum(data));
    }

    @Test
    void testFrameChecksum_isValid() {
        byte[] frame = PrismProtocol.buildConfigureFrame(1, 30, PrismMap.ColorOrder.GRB);

        // Extract payload and verify checksum
        int payloadLen = (frame[3] & 0xFF) | ((frame[4] & 0xFF) << 8);
        byte[] payload = new byte[payloadLen];
        System.arraycopy(frame, 5, payload, 0, payloadLen);

        byte expected = PrismProtocol.computeChecksum(payload);
        assertEquals(expected, frame[frame.length - 1]);
    }

    // ========================= Response Parsing Tests ========================

    @Test
    void testParseResponse_configAck() {
        // Build a CONFIG_ACK response frame manually
        byte[] payload = { 0x02, PrismMap.STATUS_OK }; // strip 2, success
        byte[] frame = buildResponseFrame(PrismMap.CMD_CONFIG_ACK, payload);

        PrismProtocol.PrismResponse response = PrismProtocol.parseResponse(frame);
        assertInstanceOf(PrismProtocol.ConfigAck.class, response);
        PrismProtocol.ConfigAck ack = (PrismProtocol.ConfigAck) response;
        assertEquals(2, ack.stripIndex());
        assertEquals(PrismMap.STATUS_OK, ack.status());
    }

    @Test
    void testParseResponse_heartbeatResponse() {
        // Build a HEARTBEAT_RSP: uptimeMs=12345 (0x3039), fwVer=0x0102, status=0
        byte[] payload = {
                0x39, 0x30, 0x00, 0x00, // uptimeMs = 12345 LE
                0x02, 0x01, // firmware version = 0x0102 LE
                0x00 // status OK
        };
        byte[] frame = buildResponseFrame(PrismMap.CMD_HEARTBEAT_RSP, payload);

        PrismProtocol.PrismResponse response = PrismProtocol.parseResponse(frame);
        assertInstanceOf(PrismProtocol.HeartbeatResponse.class, response);
        PrismProtocol.HeartbeatResponse hb = (PrismProtocol.HeartbeatResponse) response;
        assertEquals(12345, hb.uptimeMs());
        assertEquals(0x0102, hb.firmwareVersion());
        assertEquals(0, hb.status());
    }

    @Test
    void testParseResponse_nullOnInvalidSync() {
        byte[] frame = { 0x00, 0x00, PrismMap.CMD_CONFIG_ACK, 0x02, 0x00, 0x01, 0x00, 0x01 };
        assertNull(PrismProtocol.parseResponse(frame));
    }

    @Test
    void testParseResponse_nullOnTooShort() {
        assertNull(PrismProtocol.parseResponse(new byte[] { (byte) 0xAA, (byte) 0x55 }));
        assertNull(PrismProtocol.parseResponse(null));
        assertNull(PrismProtocol.parseResponse(new byte[0]));
    }

    @Test
    void testParseResponse_nullOnBadChecksum() {
        byte[] payload = { 0x01, PrismMap.STATUS_OK };
        byte[] frame = buildResponseFrame(PrismMap.CMD_CONFIG_ACK, payload);
        // Corrupt the checksum
        frame[frame.length - 1] ^= 0xFF;
        assertNull(PrismProtocol.parseResponse(frame));
    }

    @Test
    void testParseResponse_nullOnUnknownCommand() {
        byte[] payload = { 0x00 };
        byte[] frame = buildResponseFrame((byte) 0xFF, payload);
        assertNull(PrismProtocol.parseResponse(frame));
    }

    // ========================= findAndParseResponse Tests ====================

    @Test
    void testFindAndParseResponse_findsFrameInNoise() {
        byte[] payload = { 0x01, PrismMap.STATUS_OK };
        byte[] validFrame = buildResponseFrame(PrismMap.CMD_CONFIG_ACK, payload);

        // Prepend some garbage bytes
        byte[] withNoise = new byte[5 + validFrame.length];
        withNoise[0] = 0x12;
        withNoise[1] = 0x34;
        withNoise[2] = 0x56;
        withNoise[3] = 0x78;
        withNoise[4] = (byte) 0x9A;
        System.arraycopy(validFrame, 0, withNoise, 5, validFrame.length);

        PrismProtocol.PrismResponse response = PrismProtocol.findAndParseResponse(withNoise, withNoise.length);
        assertNotNull(response);
        assertInstanceOf(PrismProtocol.ConfigAck.class, response);
    }

    @Test
    void testFindAndParseResponse_nullOnEmptyBuffer() {
        assertNull(PrismProtocol.findAndParseResponse(new byte[0], 0));
    }

    // ========================= Helpers =======================================

    /** Builds a valid response frame for testing the parser. */
    private static byte[] buildResponseFrame(byte command, byte[] payload) {
        int frameLen = PrismMap.FRAME_OVERHEAD + payload.length;
        byte[] frame = new byte[frameLen];
        frame[0] = PrismMap.SYNC_BYTE_1;
        frame[1] = PrismMap.SYNC_BYTE_2;
        frame[2] = command;
        frame[3] = (byte) (payload.length & 0xFF);
        frame[4] = (byte) ((payload.length >> 8) & 0xFF);
        System.arraycopy(payload, 0, frame, 5, payload.length);
        frame[frameLen - 1] = PrismProtocol.computeChecksum(payload);
        return frame;
    }
}
