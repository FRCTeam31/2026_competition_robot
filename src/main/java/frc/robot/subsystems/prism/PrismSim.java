package frc.robot.subsystems.prism;

import com.fazecast.jSerialComm.SerialPort;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import org.prime.prism.Prism.ColorOrder;
import org.prime.prism.Prism.PrismMap;
import org.prime.prism.Protocol;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation Prism IO implementation that talks to the real Prism device
 * over a PC serial port using jSerialComm. ALL serial I/O (open, read, write)
 * happens on a single dedicated thread to work around Windows USB CDC +
 * jSerialComm thread affinity issues.
 */
public class PrismSim implements IPrism {

    private volatile boolean _connected;
    private boolean _initialized;
    private boolean _configSentAfterConnect;
    private final String _comPortName;
    private final int _baudRate;
    private double _lastHeartbeatTime;
    private double _lastHeartbeatRequestTime;
    private long _deviceUptimeMs;
    private long _lastDeviceUptimeMs;

    // Strip configuration state -- stored for resend on reconnect
    private final int[] _stripPixelCounts = new int[PrismMap.STRIP_COUNT];
    private final ColorOrder[] _stripColorOrders = new ColorOrder[PrismMap.STRIP_COUNT];
    private final boolean[] _stripConfigured = new boolean[PrismMap.STRIP_COUNT];

    // WPILib AddressableLED mirror for simulation display
    private AddressableLED _simLed;
    private AddressableLEDBuffer _simLedBuffer;
    private int _simLedLength;
    private volatile AddressableLEDBuffer[] _simLedPendingBuffers;

    // Serial thread and queues (robot loop never touches the serial port)
    private Thread _serialThread;
    private final AtomicBoolean _serialRunning = new AtomicBoolean(false);
    private final ConcurrentLinkedQueue<byte[]> _commandQueue = new ConcurrentLinkedQueue<>();
    private volatile byte[] _latestPixelFrame; // Only keep the most recent pixel frame
    private final ConcurrentLinkedQueue<Protocol.PrismResponse> _responseQueue = new ConcurrentLinkedQueue<>();

    public PrismSim(String comPortName, int baudRate) {
        _connected = false;
        _initialized = false;
        _configSentAfterConnect = false;
        _comPortName = comPortName;
        _baudRate = baudRate;
        _lastHeartbeatTime = 0;
        _lastHeartbeatRequestTime = 0;
        _deviceUptimeMs = 0;
        _lastDeviceUptimeMs = 0;
    }

    // ========================= IPrism Implementation ========================

    @Override
    public void updateInputs(PrismInputsAutoLogged inputs) {
        inputs.Connected = _connected;
        inputs.DeviceUptimeMs = _deviceUptimeMs;
    }

    @Override
    public boolean configureStrip(int strip, int pixelCount, ColorOrder order) {
        _stripPixelCounts[strip] = pixelCount;
        _stripColorOrders[strip] = order;
        _stripConfigured[strip] = true;

        if (!_connected) {
            return false;
        }

        byte[] frame = Protocol.buildConfigureFrame(strip, pixelCount, order);

        for (int attempt = 0; attempt < PrismMap.MAX_RETRIES; attempt++) {
            _commandQueue.add(frame);
            long deadline = System.currentTimeMillis() + PrismMap.CONFIG_ACK_TIMEOUT_MS;
            while (System.currentTimeMillis() < deadline) {
                Protocol.PrismResponse response = _responseQueue.poll();
                if (response instanceof Protocol.ConfigAck ack
                        && ack.stripIndex() == strip
                        && ack.status() == PrismMap.STATUS_OK) {
                    return true;
                }
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    return false;
                }
            }
            DataLogManager.log("[PrismSim] Config attempt " + (attempt + 1) + " timed out for strip " + strip);
        }

        DataLogManager
                .log("[PrismSim] Failed to configure strip " + strip + " after " + PrismMap.MAX_RETRIES + " attempts");
        return false;
    }

    @Override
    public boolean sendPixelData(AddressableLEDBuffer[] buffers) {
        if (!_connected) {
            return false;
        }

        try {
            byte[] frame = Protocol.buildPixelDataAllFrame(buffers);
            _latestPixelFrame = frame;

            // Stash buffers for the sim LED mirror (updated in periodicHeartbeat)
            _simLedPendingBuffers = buffers;

            return true;
        } catch (Exception e) {
            DataLogManager.log("[PrismSim] Error building pixel data: " + e.getMessage());
            return false;
        }
    }

    @Override
    public void periodicHeartbeat() {
        // Lazy-init: start the serial thread on first call
        if (!_initialized) {
            _initialized = true;
            startSerialThread();
        }

        double now = Timer.getFPGATimestamp();

        if (!_connected) {
            return;
        }

        // Enqueue heartbeat request periodically
        if (now - _lastHeartbeatRequestTime >= PrismMap.HEARTBEAT_INTERVAL_MS / 1000.0) {
            _lastHeartbeatRequestTime = now;
            _commandQueue.add(Protocol.buildHeartbeatRequest());
            if (_lastHeartbeatTime == 0) {
                _lastHeartbeatTime = now;
            }
        }

        // Drain all responses from the serial thread
        Protocol.PrismResponse response;
        while ((response = _responseQueue.poll()) != null) {
            if (response instanceof Protocol.HeartbeatResponse hb) {
                _lastHeartbeatTime = now;
                _lastDeviceUptimeMs = _deviceUptimeMs;
                _deviceUptimeMs = hb.uptimeMs();

                // Send strip configs once we have a live connection
                if (!_configSentAfterConnect) {
                    _configSentAfterConnect = true;
                    DataLogManager.log("[PrismSim] First heartbeat received — sending strip configurations.");
                    resendAllConfigurations();
                }

                if (_deviceUptimeMs < _lastDeviceUptimeMs && _lastDeviceUptimeMs > 0) {
                    DataLogManager.log("[PrismSim] Device reboot detected. Resending configuration.");
                    resendAllConfigurations();
                }
            }
        }

        // Check for heartbeat timeout (generous for USB CDC)
        double simHeartbeatTimeoutS = (PrismMap.HEARTBEAT_TIMEOUT_MS * 3) / 1000.0;
        if (_lastHeartbeatTime > 0 && (now - _lastHeartbeatTime) > simHeartbeatTimeoutS) {
            DataLogManager.log("[PrismSim] Heartbeat timeout -- marking disconnected. (delta="
                    + (now - _lastHeartbeatTime) + "s)");
            _connected = false;
        }

        // Update the simulation LED mirror (deferred from sendPixelData)
        AddressableLEDBuffer[] pending = _simLedPendingBuffers;
        if (pending != null) {
            _simLedPendingBuffers = null;
            updateSimLed(pending);
        }
    }

    @Override
    public boolean isConnected() {
        return _connected;
    }

    @Override
    public void close() {
        _serialRunning.set(false);
        if (_serialThread != null) {
            _serialThread.interrupt();
            try {
                _serialThread.join(2000);
            } catch (InterruptedException ignored) {
            }
            _serialThread = null;
        }
        if (_simLed != null) {
            _simLed.close();
            _simLed = null;
        }
        _connected = false;
    }

    // ========================= Serial Thread ================================

    private void startSerialThread() {
        _serialRunning.set(true);
        _serialThread = new Thread(() -> {
            SerialPort port = null;
            try {
                port = SerialPort.getCommPort(_comPortName);
                port.setBaudRate(_baudRate);
                port.setNumDataBits(PrismMap.DATA_BITS);
                port.setNumStopBits(PrismMap.STOP_BITS);
                port.setParity(SerialPort.NO_PARITY);
                port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 5, 0);
                port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);

                if (!port.openPort()) {
                    DriverStation.reportError("[PrismSim] Failed to open port " + _comPortName, false);
                    return;
                }

                port.setDTR();
                port.setRTS();

                // ESP32-S3 reboots on port open
                Thread.sleep(2000);

                // Drain boot garbage
                byte[] drain = new byte[256];
                int totalDrained = 0;
                while (true) {
                    int n = port.readBytes(drain, drain.length);
                    if (n > 0) {
                        totalDrained += n;
                    } else {
                        break;
                    }
                }
                if (totalDrained > 0) {
                    DataLogManager.log("[PrismSim] Drained " + totalDrained + " boot bytes");
                }

                _connected = true;
                DataLogManager.log("[PrismSim] Connected on " + _comPortName);

                // Main I/O loop — all reads and writes happen here
                byte[] readBuf = new byte[64];
                byte[] accum = new byte[256];
                int accumLen = 0;

                while (_serialRunning.get()) {
                    // 1) Read first — never starve reads
                    int space = accum.length - accumLen;
                    if (space <= 0) {
                        accumLen = 0;
                        space = accum.length;
                    }
                    int n = port.readBytes(readBuf, Math.min(readBuf.length, space));
                    if (n > 0) {
                        System.arraycopy(readBuf, 0, accum, accumLen, n);
                        accumLen += n;

                        // Parse all complete frames in the buffer
                        boolean parsed = true;
                        while (parsed && accumLen > 0) {
                            parsed = false;
                            Protocol.PrismResponse resp = Protocol.findAndParseResponse(accum, accumLen);
                            if (resp != null) {
                                _responseQueue.add(resp);
                                parsed = true;

                                int frameSize = getFrameSize(resp);
                                int frameStart = findSyncOffset(accum, accumLen);
                                if (frameStart >= 0) {
                                    int consumed = frameStart + frameSize;
                                    int remaining = accumLen - consumed;
                                    if (remaining > 0) {
                                        System.arraycopy(accum, consumed, accum, 0, remaining);
                                    }
                                    accumLen = Math.max(remaining, 0);
                                } else {
                                    accumLen = 0;
                                }
                            }
                        }
                    }

                    // 2) Write queued commands (heartbeat, config — small frames)
                    byte[] toWrite;
                    while ((toWrite = _commandQueue.poll()) != null) {
                        port.writeBytes(toWrite, toWrite.length);
                    }

                    // 3) Send latest pixel frame (at most once per loop, drop stale)
                    byte[] pixelFrame = _latestPixelFrame;
                    _latestPixelFrame = null;
                    if (pixelFrame != null) {
                        port.writeBytes(pixelFrame, pixelFrame.length);
                    }
                }
            } catch (InterruptedException ignored) {
                // Normal shutdown
            } catch (Exception e) {
                if (_serialRunning.get()) {
                    DataLogManager.log("[PrismSim] Serial thread error: " + e.getMessage());
                }
            } finally {
                _connected = false;
                if (port != null) {
                    try {
                        port.closePort();
                    } catch (Exception ignored) {
                    }
                }
                DataLogManager.log("[PrismSim] Serial thread exited");
            }
        }, "PrismSim-IO");
        _serialThread.setDaemon(true);
        _serialThread.start();
    }

    // ========================= Helpers ======================================

    /**
     * Mirrors the pixel data to a WPILib AddressableLED so it appears in the
     * simulation GUI. All strip buffers are concatenated into a single LED string.
     */
    private void updateSimLed(AddressableLEDBuffer[] buffers) {
        int totalLength = 0;
        for (AddressableLEDBuffer buf : buffers) {
            totalLength += buf.getLength();
        }

        // (Re)create the AddressableLED if the total length changed
        if (_simLed == null || totalLength != _simLedLength) {
            if (_simLed != null) {
                _simLed.close();
            }
            _simLedLength = totalLength;
            _simLedBuffer = new AddressableLEDBuffer(totalLength);
            _simLed = new AddressableLED(frc.robot.subsystems.prism.PrismMap.SIM_LED_PWM_PORT);
            _simLed.setLength(totalLength);
            _simLed.start();
        }

        // Copy pixel data from all strip buffers into the combined buffer
        int offset = 0;
        for (AddressableLEDBuffer buf : buffers) {
            for (int i = 0; i < buf.getLength(); i++) {
                _simLedBuffer.setLED(offset + i, buf.getLED(i));
            }
            offset += buf.getLength();
        }

        _simLed.setData(_simLedBuffer);
    }

    private static int getFrameSize(Protocol.PrismResponse resp) {
        if (resp instanceof Protocol.HeartbeatResponse) {
            return PrismMap.FRAME_OVERHEAD + 7;
        } else if (resp instanceof Protocol.ConfigAck) {
            return PrismMap.FRAME_OVERHEAD + 2;
        }
        return PrismMap.FRAME_OVERHEAD;
    }

    private void resendAllConfigurations() {
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            if (_stripConfigured[i]) {
                configureStrip(i, _stripPixelCounts[i], _stripColorOrders[i]);
            }
        }
    }

    private static int findSyncOffset(byte[] buf, int len) {
        for (int i = 0; i <= len - 2; i++) {
            if (buf[i] == (byte) 0xAA && buf[i + 1] == (byte) 0x55) {
                return i;
            }
        }
        return -1;
    }
}
