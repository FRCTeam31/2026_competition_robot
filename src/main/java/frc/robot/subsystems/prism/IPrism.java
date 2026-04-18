package frc.robot.subsystems.prism;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.prime.prism.Prism.ColorOrder;

/**
 * Interface for Prism device IO. Real implementation uses WPILib SerialPort (roboRIO),
 * Sim implementation uses jSerialComm to talk to the real device from the PC.
 */
public interface IPrism {

    /**
     * Updates the inputs for logging.
     */
    void updateInputs(PrismInputsAutoLogged inputs);

    /**
     * Configures a strip on the device.
     *
     * @param strip      Strip index (0-3)
     * @param pixelCount Number of pixels
     * @param order      Color order
     * @return true if acknowledged
     */
    boolean configureStrip(int strip, int pixelCount, ColorOrder order);

    /**
     * Sends pixel data for all strips.
     *
     * @param buffers Array of 4 AddressableLEDBuffers
     * @return true if write succeeded
     */
    boolean sendPixelData(AddressableLEDBuffer[] buffers);

    /**
     * Periodic heartbeat / health check. Call from update loop.
     */
    void periodicHeartbeat();

    /**
     * @return true if device is connected
     */
    boolean isConnected();

    /**
     * Close the connection and release resources.
     */
    void close();
}
