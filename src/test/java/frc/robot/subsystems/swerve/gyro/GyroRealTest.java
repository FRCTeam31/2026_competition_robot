package frc.robot.subsystems.swerve.gyro;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;

/**
 * Unit tests for GyroReal - documents hardware integration tests.
 * GyroReal wraps Pigeon2 hardware which cannot be meaningfully tested without actual hardware.
 * These tests verify interface compliance and document expected hardware behavior.
 */
class GyroRealTest {
    private GyroInputsAutoLogged inputs;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        inputs = new GyroInputsAutoLogged();
    }

    @Test
    void testImplementsIGyroInterface() {
        assertTrue(IGyro.class.isAssignableFrom(GyroReal.class));
    }

    @Test
    void testHasRequiredMethods() throws NoSuchMethodException {
        GyroReal.class.getDeclaredMethod("updateInputs", GyroInputsAutoLogged.class, double.class);
        GyroReal.class.getDeclaredMethod("reset");
        GyroReal.class.getDeclaredMethod("reset", double.class);
    }

    @Test
    void testInputsStructureExists() {
        assertNotNull(inputs);
    }
}
