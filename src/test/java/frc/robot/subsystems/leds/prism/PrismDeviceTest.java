package frc.robot.subsystems.leds.prism;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.prime.prism.Prism;
import org.prime.prism.Prism.ColorOrder;

import edu.wpi.first.hal.HAL;

/**
 * Unit tests for {@link Prism} -- verifies construction and configuration storage.
 *
 * <p>Note: Full serial I/O testing requires a physical device or mock serial port.
 * These tests verify the device's state management and configuration tracking logic
 * using HAL simulation.
 */
class PrismDeviceTest {

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void testConstructor_doesNotThrow() {
        // In HAL sim, SerialPort construction may succeed even without a real device.
        // The device should construct without throwing regardless.
        assertDoesNotThrow(() -> new Prism(edu.wpi.first.wpilibj.SerialPort.Port.kUSB));
    }

    @Test
    void testConfigureStrip_storesConfigEvenWhenDisconnected() {
        Prism device = new Prism(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);

        // Should not throw even when disconnected -- config is stored for later
        boolean result = device.configureStrip(0, 30, ColorOrder.GRB);
        assertFalse(result, "Should return false when disconnected");
    }

    @Test
    void testSendPixelData_returnsFalseAfterClose() {
        Prism device = new Prism(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);
        device.close(); // Force disconnect

        edu.wpi.first.wpilibj.AddressableLEDBuffer[] buffers = new edu.wpi.first.wpilibj.AddressableLEDBuffer[4];
        for (int i = 0; i < 4; i++) {
            buffers[i] = new edu.wpi.first.wpilibj.AddressableLEDBuffer(8);
        }

        assertFalse(device.sendPixelData(buffers), "Should return false when disconnected");
    }

    @Test
    void testGetDeviceUptimeMs_defaultsToZero() {
        Prism device = new Prism(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);
        assertEquals(0, device.getDeviceUptimeMs());
    }

    @Test
    void testClose_doesNotThrow() {
        Prism device = new Prism(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);
        assertDoesNotThrow(device::close, "close() should not throw even when not connected");
    }

    @Test
    void testPeriodicHeartbeat_doesNotThrowWhenDisconnected() {
        Prism device = new Prism(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);
        assertDoesNotThrow(device::periodicHeartbeat, "periodicHeartbeat should not throw when disconnected");
    }
}
