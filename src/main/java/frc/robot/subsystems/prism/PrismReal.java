package frc.robot.subsystems.prism;

import org.prime.prism.Prism;
import org.prime.prism.Prism.ColorOrder;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * Real Prism IO implementation. Delegates to the existing {@link Prism} class
 * which uses WPILib's {@link SerialPort} for communication on the roboRIO.
 */
public class PrismReal implements IPrism {

    private final Prism _device;

    public PrismReal(SerialPort.Port port) {
        _device = new Prism(port);
    }

    @Override
    public void updateInputs(PrismInputsAutoLogged inputs) {
        inputs.Connected = _device.isConnected();
        inputs.DeviceUptimeMs = _device.getDeviceUptimeMs();
    }

    @Override
    public boolean configureStrip(int strip, int pixelCount, ColorOrder order) {
        return _device.configureStrip(strip, pixelCount, order);
    }

    @Override
    public boolean sendPixelData(AddressableLEDBuffer[] buffers) {
        return _device.sendPixelData(buffers);
    }

    @Override
    public void periodicHeartbeat() {
        _device.periodicHeartbeat();
    }

    @Override
    public boolean isConnected() {
        return _device.isConnected();
    }

    @Override
    public void close() {
        _device.close();
    }
}
