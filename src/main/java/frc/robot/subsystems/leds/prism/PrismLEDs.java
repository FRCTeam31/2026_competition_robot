package frc.robot.subsystems.leds.prism;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.ILEDs;

/**
 * LED subsystem that drives a Prism USB serial LED controller.
 *
 * <p>Renders all patterns locally using WPILib's {@link LEDPattern} at 125fps,
 * then streams the raw RGB pixel data to the Prism device over USB serial.
 * Local {@link AddressableLEDBuffer}s are maintained for simulation, dashboard
 * readback, and other local consumers.
 */
public class PrismLEDs extends SubsystemBase implements ILEDs {

    /**
     * Configuration for a single LED strip on the Prism device.
     */
    public record StripConfig(int pixelCount, PrismMap.ColorOrder colorOrder) {
    }

    private final ScheduledExecutorService _updateLoopExecutor = Executors.newScheduledThreadPool(1);
    private final PrismDevice _device;
    private final AddressableLEDBuffer[] _buffers;
    private final AddressableLEDBufferView[] _sections;
    private final LEDPattern[] _sectionPatterns;
    private byte _loopErrorCounter = 0;

    private final LEDPattern _initialPattern = LEDPattern.solid(Color.kGhostWhite).breathe(Units.Seconds.of(4));
    private final Alert _loopStoppedAlert = new Alert("[Prism] Update loop failed.", Alert.AlertType.kWarning);

    /**
     * Creates a PrismLEDs subsystem connected to the specified USB port.
     *
     * @param port         The serial port to use (e.g., {@link SerialPort.Port#kUSB})
     * @param stripConfigs Configuration for each of the 4 strips. Array length must equal
     *                     {@link PrismMap#STRIP_COUNT}.
     */
    public PrismLEDs(SerialPort.Port port, StripConfig[] stripConfigs) {
        if (stripConfigs.length != PrismMap.STRIP_COUNT) {
            throw new IllegalArgumentException(
                    "Expected " + PrismMap.STRIP_COUNT + " strip configs, got " + stripConfigs.length);
        }

        // Create device and configure strips
        _device = new PrismDevice(port);

        _buffers = new AddressableLEDBuffer[PrismMap.STRIP_COUNT];
        _sections = new AddressableLEDBufferView[PrismMap.STRIP_COUNT];
        _sectionPatterns = new LEDPattern[PrismMap.STRIP_COUNT];

        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            var config = stripConfigs[i];

            // Configure device strip
            _device.configureStrip(i, config.pixelCount(), config.colorOrder());

            // Create local buffer for rendering and readback
            _buffers[i] = new AddressableLEDBuffer(config.pixelCount());
            _sections[i] = _buffers[i].createView(0, config.pixelCount() - 1);
            _sectionPatterns[i] = _initialPattern;
        }

        // Apply initial pattern
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            _initialPattern.applyTo(_buffers[i]);
        }

        // Start 125fps update loop
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLoop, 0, 8, TimeUnit.MILLISECONDS);
        _loopStoppedAlert.set(false);
    }

    // ========================= Update Loop ==================================

    private void updateLoop() {
        if (_loopErrorCounter > PrismMap.MAX_RETRIES) {
            _loopStoppedAlert.set(true);
            stopUpdateLoop();
            return;
        }

        _loopStoppedAlert.set(false);

        try {
            // Render patterns to local buffers
            for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
                _sectionPatterns[i].applyTo(_sections[i]);
            }

            // Stream pixel data to device
            if (_device.isConnected()) {
                _device.sendPixelData(_buffers);
            }

            // Periodic heartbeat / reconnect
            _device.periodicHeartbeat();
        } catch (Exception e) {
            _loopErrorCounter++;
            DataLogManager.log("[Prism:ERROR] Update loop failed: " + e.getMessage());
            DriverStation.reportError("[Prism:ERROR] Update loop failed: " + e.getMessage(),
                    e.getStackTrace());
        }
    }

    // ========================= Loop Control =================================

    /**
     * Stops the update loop for the LEDs.
     */
    public void stopUpdateLoop() {
        _updateLoopExecutor.shutdown();
        _loopStoppedAlert.set(true);
    }

    /**
     * Starts the update loop for the LEDs.
     */
    public void startUpdateLoop() {
        _loopErrorCounter = 0;
        _updateLoopExecutor.scheduleAtFixedRate(this::updateLoop, 0, 8, TimeUnit.MILLISECONDS);
    }

    // ========================= ILEDs Implementation =========================

    @Override
    public void setSectionPattern(int section, LEDPattern pattern) {
        _sectionPatterns[section] = pattern;
    }

    @Override
    public void setAllSectionPatterns(LEDPattern pattern) {
        for (int i = 0; i < PrismMap.STRIP_COUNT; i++) {
            _sectionPatterns[i] = pattern;
        }
    }

    @Override
    public Command setSectionPatternCommand(int section, LEDPattern pattern) {
        return Commands.runOnce(() -> setSectionPattern(section, pattern)).ignoringDisable(true);
    }

    @Override
    public Command setAllSectionPatternsCommand(LEDPattern pattern) {
        return Commands.runOnce(() -> setAllSectionPatterns(pattern)).ignoringDisable(true);
    }

    // ========================= Accessors ====================================

    /**
     * Returns the local LED buffer for a given strip. Useful for simulation,
     * dashboard rendering, or reading back rendered pixel data.
     *
     * @param strip Strip index (0-3)
     * @return The local AddressableLEDBuffer for this strip
     */
    public AddressableLEDBuffer getBuffer(int strip) {
        return _buffers[strip];
    }

    /**
     * @return true if the Prism device is connected and responding to heartbeats
     */
    public boolean isDeviceConnected() {
        return _device.isConnected();
    }
}
