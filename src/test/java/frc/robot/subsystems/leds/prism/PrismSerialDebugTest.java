package frc.robot.subsystems.leds.prism;

import com.fazecast.jSerialComm.SerialPort;

/**
 * Standalone serial debug test — run this directly (not through WPILib sim)
 * to verify basic serial communication with the Prism device.
 *
 * Usage: Run the main() method directly from your IDE.
 */
public class PrismSerialDebugTest {

    private static String bytesToHex(byte[] bytes, int len) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < len; i++) {
            if (sb.length() > 0)
                sb.append(' ');
            sb.append(String.format("%02X", bytes[i] & 0xFF));
        }
        return sb.toString();
    }

    public static void main(String[] args) throws Exception {
        String portName = "COM12";

        System.out.println("=== Prism Serial Debug Test ===");
        System.out.println("Available ports:");
        for (SerialPort p : SerialPort.getCommPorts()) {
            System.out.println("  " + p.getSystemPortName() + " — " + p.getDescriptivePortName());
        }
        System.out.println();

        SerialPort port = SerialPort.getCommPort(portName);
        port.setBaudRate(115200);
        port.setNumDataBits(8);
        port.setNumStopBits(1);
        port.setParity(SerialPort.NO_PARITY);
        port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);

        // Try different timeout modes
        port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 1000, 0);

        System.out.println("Opening " + portName + "...");
        if (!port.openPort()) {
            System.err.println("FAILED to open port!");
            return;
        }
        System.out.println("Port opened. DTR=" + port.getDTR() + " RTS=" + port.getRTS());

        // Assert DTR/RTS
        port.setDTR();
        port.setRTS();
        System.out.println("After set: DTR=" + port.getDTR() + " RTS=" + port.getRTS());

        // Wait for ESP32 to boot
        System.out.println("Waiting 2s for device to boot...");
        Thread.sleep(2000);

        // Drain any boot output
        int stale = port.bytesAvailable();
        if (stale > 0) {
            byte[] drain = new byte[stale];
            port.readBytes(drain, stale);
            System.out.println("Drained " + stale + " stale bytes: " + bytesToHex(drain, stale));
        }

        // Build heartbeat request: [0xAA][0x55][0x04][0x00][0x00][0x00]
        byte[] heartbeat = new byte[] { (byte) 0xAA, 0x55, 0x04, 0x00, 0x00, 0x00 };

        for (int attempt = 1; attempt <= 5; attempt++) {
            System.out.println("\n--- Attempt " + attempt + " ---");
            int written = port.writeBytes(heartbeat, heartbeat.length);
            System.out.println("Wrote " + written + " bytes: " + bytesToHex(heartbeat, heartbeat.length));

            // Wait and read with blocking timeout (1s)
            Thread.sleep(100); // Give device time to process
            int avail = port.bytesAvailable();
            System.out.println("Bytes available: " + avail);

            if (avail > 0) {
                byte[] buf = new byte[64];
                int read = port.readBytes(buf, Math.min(avail, buf.length));
                System.out.println("Read " + read + " bytes: " + bytesToHex(buf, read));
            } else {
                // Try a blocking read anyway
                System.out.println("No bytes available, trying blocking read (1s timeout)...");
                byte[] buf = new byte[64];
                int read = port.readBytes(buf, buf.length);
                System.out.println("Blocking read got " + read + " bytes: " + bytesToHex(buf, read));
            }

            Thread.sleep(500);
        }

        port.closePort();
        System.out.println("\nPort closed. Done.");
    }
}
