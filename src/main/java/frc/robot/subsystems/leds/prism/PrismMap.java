package frc.robot.subsystems.leds.prism;

/**
 * Constants for the Prism USB serial LED controller.
 */
public class PrismMap {

    // ========================= Serial Configuration =========================

    public static final int BAUD_RATE = 2_000_000;
    public static final int DATA_BITS = 8;
    public static final int STOP_BITS = 1;

    // ========================= Frame Framing ================================

    public static final byte SYNC_BYTE_1 = (byte) 0xAA;
    public static final byte SYNC_BYTE_2 = (byte) 0x55;

    // ========================= Commands (RoboRIO -> Device) ==================

    public static final byte CMD_CONFIGURE = 0x01;
    public static final byte CMD_PIXEL_DATA = 0x02;
    public static final byte CMD_PIXEL_DATA_ALL = 0x03;
    public static final byte CMD_HEARTBEAT_REQ = 0x04;

    // ========================= Commands (Device -> RoboRIO) ==================

    public static final byte CMD_CONFIG_ACK = (byte) 0x81;
    public static final byte CMD_HEARTBEAT_RSP = (byte) 0x84;

    // ========================= Status Codes =================================

    public static final byte STATUS_OK = 0x00;
    public static final byte STATUS_ERROR = 0x01;

    // ========================= Strip Configuration ==========================

    public static final int STRIP_COUNT = 4;
    public static final int MAX_PIXELS_PER_STRIP = 144;

    // ========================= Timeouts & Retry =============================

    public static final int CONFIG_ACK_TIMEOUT_MS = 100;
    public static final int HEARTBEAT_TIMEOUT_MS = 500;
    public static final int MAX_RETRIES = 3;
    public static final int RECONNECT_INTERVAL_MS = 1000;
    public static final int HEARTBEAT_INTERVAL_MS = 250;

    // ========================= Frame Overhead ===============================

    /** 2 sync bytes + 1 command + 2 length + 1 checksum = 6 bytes framing overhead */
    public static final int FRAME_OVERHEAD = 6;

    // ========================= Color Order Enum =============================

    public enum ColorOrder {
        RGB((byte) 0x00),
        GRB((byte) 0x01),
        RGBW((byte) 0x02),
        GRBW((byte) 0x03);

        public final byte value;

        ColorOrder(byte value) {
            this.value = value;
        }
    }
}
