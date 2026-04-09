package frc.robot.subsystems.leds.prism;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * Manages serial communication with a single Prism USB LED controller device.
 *
 * <p>Handles strip configuration (with ACK + retry), pixel data streaming,
 * heartbeat health monitoring, and automatic reconnection on disconnect.
 */
public class PrismDevice {
    private SerialPort _serialPort;
    private final SerialPort.Port _port;
    private boolean _connected;
    private double _lastHeartbeatTime;
    private double _lastHeartbeatRequestTime;
    private long _deviceUptimeMs;
    private long _lastDeviceUptimeMs;

    // Strip configuration state — stored for resend on reconnect
    private final int[] _stripPixelCounts = new int[PrismMap.STRIP_COUNT];
    private final PrismMap.ColorOrder[] _stripColorOrders = new PrismMap.ColorOrder[PrismMap.STRIP_COUNT];
    private final boolean[] _stripConfigured = new boolean[PrismMap.STRIP_COUNT];

    private final Alert _disconnectedAlert = new Alert("[Prism] Device disconnected.", Alert.AlertType.kWarning);

    // Read buffer for responses
    private final byte[] _readBuffer = new byte[64];

    /**
     * Creates a new PrismDevice and opens the serial port.
     *
     * @param port The USB serial port to use (e.g., SerialPort.Port.kUSB)
     */
    public PrismDevice(SerialPort.Port port) {
        _port = port;
        _connected = false;
        _lastHeartbeatTime = 0;
        _lastHeartbeatRequestTime = 0;
        _deviceUptimeMs = 0;
        _lastDeviceUptimeMs = 0;

        tryConnect();
    }

    // ========================= Strip Configuration ==========================

    /**
     * Configures a strip on the device. Sends the configuration frame and
     * waits for acknowledgement with timeout and retry.
     *
     * @param strip      Strip index (0-3)
     * @param pixelCount Number of pixels on this strip
     * @param order      Color order for the strip's LEDs
     * @return true if the device acknowledged the configuration
     */
    public boolean configureStrip(int strip, int pixelCount, PrismMap.ColorOrder order) {
        // Store config for reconnect
        _stripPixelCounts[strip] = pixelCount;
        _stripColorOrders[strip] = order;
        _stripConfigured[strip] = true;

        if (!_connected) {
            return false;
        }

        byte[] frame = PrismProtocol.buildConfigureFrame(strip, pixelCount, order);

        for (int attempt = 0; attempt < PrismMap.MAX_RETRIES; attempt++) {
            try {
                _serialPort.write(frame, frame.length);
                _serialPort.flush();

                // Wait for CONFIG_ACK
                PrismProtocol.PrismResponse response = readResponse(PrismMap.CONFIG_ACK_TIMEOUT_MS);
                if (response instanceof PrismProtocol.ConfigAck ack
                        && ack.stripIndex() == strip
                        && ack.status() == PrismMap.STATUS_OK) {
                    return true;
                }
            } catch (Exception e) {
                DataLogManager.log("[Prism] Config attempt " + (attempt + 1) + " failed: " + e.getMessage());
            }
        }

        DataLogManager
                .log("[Prism] Failed to configure strip " + strip + " after " + PrismMap.MAX_RETRIES + " attempts");
        return false;
    }

    // ========================= Pixel Data ===================================

    /**
     * Sends pixel data for all strips to the device.
     *
     * @param buffers Array of 4 AddressableLEDBuffers (one per strip)
     * @return true if the write succeeded
     */
    public boolean sendPixelData(AddressableLEDBuffer[] buffers) {
        if (!_connected) {
            return false;
        }

        try {
            byte[] frame = PrismProtocol.buildPixelDataAllFrame(buffers);
            _serialPort.write(frame, frame.length);
            return true;
        } catch (Exception e) {
            handleSerialError("sendPixelData", e);
            return false;
        }
    }

    // ========================= Health Monitoring ============================

    /**
     * Sends a heartbeat request if enough time has elapsed since the last one.
     * Should be called periodically from the subsystem's update loop.
     */
    public void periodicHeartbeat() {
        double now = Timer.getFPGATimestamp();

        if (!_connected) {
            // Try to reconnect periodically
            if (now - _lastHeartbeatRequestTime >= PrismMap.RECONNECT_INTERVAL_MS / 1000.0) {
                _lastHeartbeatRequestTime = now;
                tryReconnect();
            }
            return;
        }

        // Send heartbeat request periodically
        if (now - _lastHeartbeatRequestTime >= PrismMap.HEARTBEAT_INTERVAL_MS / 1000.0) {
            _lastHeartbeatRequestTime = now;
            try {
                byte[] frame = PrismProtocol.buildHeartbeatRequest();
                _serialPort.write(frame, frame.length);
            } catch (Exception e) {
                handleSerialError("heartbeat request", e);
                return;
            }
        }

        // Check for heartbeat responses (non-blocking)
        try {
            int available = _serialPort.getBytesReceived();
            if (available > 0) {
                int toRead = Math.min(available, _readBuffer.length);
                byte[] data = _serialPort.read(toRead);
                PrismProtocol.PrismResponse response = PrismProtocol.findAndParseResponse(data, data.length);

                if (response instanceof PrismProtocol.HeartbeatResponse hb) {
                    _lastHeartbeatTime = now;
                    _lastDeviceUptimeMs = _deviceUptimeMs;
                    _deviceUptimeMs = hb.uptimeMs();

                    // Detect device reboot (uptime reset)
                    if (_deviceUptimeMs < _lastDeviceUptimeMs && _lastDeviceUptimeMs > 0) {
                        DataLogManager.log("[Prism] Device reboot detected. Resending configuration.");
                        resendAllConfigurations();
                    }
                }
            }
        } catch (Exception e) {
            handleSerialError("heartbeat read", e);
            return;
        }

        // Check for heartbeat timeout
        if (_lastHeartbeatTime > 0 && (now - _lastHeartbeatTime) > PrismMap.HEARTBEAT_TIMEOUT_MS / 1000.0) {
            DataLogManager.log("[Prism] Heartbeat timeout — marking device disconnected.");
            markDisconnected();
        }
    }

    /**
     * @return true if the device is considered connected (heartbeat received recently)
     */
    public boolean isConnected() {
        return _connected;
    }

    /**
     * @return The last known device uptime in milliseconds
     */
    public long getDeviceUptimeMs() {
        return _deviceUptimeMs;
    }

    // ========================= Connection Management ========================

    /**
     * Closes the serial port and releases resources.
     */
    public void close() {
        if (_serialPort != null) {
            try {
                _serialPort.close();
            } catch (Exception e) {
                // Ignore close errors
            }
            _serialPort = null;
        }
        _connected = false;
        _disconnectedAlert.set(true);
    }

    // ========================= Internal =====================================

    private void tryConnect() {
        try {
            _serialPort = new SerialPort(PrismMap.BAUD_RATE, _port);
            _serialPort.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
            _serialPort.setTimeout(0.01); // 10ms read timeout
            _connected = true;
            _lastHeartbeatTime = Timer.getFPGATimestamp();
            _disconnectedAlert.set(false);
            DataLogManager.log("[Prism] Connected on port " + _port);
        } catch (Exception e) {
            _connected = false;
            _disconnectedAlert.set(true);
            DriverStation.reportError("[Prism] Failed to open serial port: " + e.getMessage(), false);
        }
    }

    private void tryReconnect() {
        close();
        tryConnect();

        if (_connected) {
            DataLogManager.log("[Prism] Reconnected. Resending configuration.");
            resendAllConfigurations();
        }
    }

    private void resendAllConfigurations() {
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            if (_stripConfigured[i]) {
                configureStrip(i, _stripPixelCounts[i], _stripColorOrders[i]);
            }
        }
    }

    private PrismProtocol.PrismResponse readResponse(int timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;

        while (System.currentTimeMillis() < deadline) {
            try {
                int available = _serialPort.getBytesReceived();
                if (available > 0) {
                    int toRead = Math.min(available, _readBuffer.length);
                    byte[] data = _serialPort.read(toRead);
                    PrismProtocol.PrismResponse response = PrismProtocol.findAndParseResponse(data, data.length);
                    if (response != null) {
                        return response;
                    }
                }
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            } catch (Exception e) {
                return null;
            }
        }
        return null;
    }

    private void handleSerialError(String context, Exception e) {
        DataLogManager.log("[Prism] Serial error during " + context + ": " + e.getMessage());
        markDisconnected();
    }

    private void markDisconnected() {
        _connected = false;
        _disconnectedAlert.set(true);
    }
}
