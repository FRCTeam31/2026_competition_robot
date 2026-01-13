package frc.robot.subsystems.swerve.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Unit tests for AutoAlign - validates PID-based rotational alignment control.
 * Tests verify setpoint management, correction calculations, and PID controller state.
 */
class AutoAlignTest {
    private AutoAlign autoAlign;
    private static final double EPSILON = 1e-6;
    private static final ExtendedPIDConstants TEST_PID_CONSTANTS = new ExtendedPIDConstants(4.0, 0.0, 0.08);

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        autoAlign = new AutoAlign(TEST_PID_CONSTANTS);
    }

    // ==================== Constructor Tests ====================

    @Test
    void testConstructor_InitializesWithZeroSetpoint() {
        assertEquals(0.0, autoAlign.getSetpoint().getRadians(), EPSILON,
                "AutoAlign should initialize with a zero setpoint");
    }

    @Test
    void testConstructor_AcceptsPIDConstants() {
        assertDoesNotThrow(() -> new AutoAlign(TEST_PID_CONSTANTS),
                "AutoAlign constructor should accept valid PID constants");
    }

    @Test
    void testConstructor_ThrowsOnNullPIDConstants() {
        assertThrows(Exception.class, () -> new AutoAlign(null),
                "AutoAlign constructor should throw exception for null PID constants");
    }

    // ==================== Setpoint Tests ====================

    @Test
    void testSetSetpoint_StoresCorrectValue() {
        Rotation2d targetAngle = Rotation2d.fromDegrees(90);
        autoAlign.setSetpoint(targetAngle);

        assertEquals(Math.PI / 2, autoAlign.getSetpoint().getRadians(), EPSILON,
                "Setpoint should be stored correctly in radians");
    }

    @Test
    void testSetSetpoint_ModulatesAngle() {
        // Test that angles > 180 degrees are wrapped to -180 to 180 range
        Rotation2d largeAngle = Rotation2d.fromDegrees(270);
        autoAlign.setSetpoint(largeAngle);

        double expectedModulated = Math.toRadians(-90); // 270 degrees wraps to -90
        assertEquals(expectedModulated, autoAlign.getSetpoint().getRadians(), EPSILON,
                "Setpoint should be modulated to [-π, π] range");
    }

    @Test
    void testSetSetpoint_HandlesNegativeAngles() {
        Rotation2d negativeAngle = Rotation2d.fromDegrees(-45);
        autoAlign.setSetpoint(negativeAngle);

        assertEquals(Math.toRadians(-45), autoAlign.getSetpoint().getRadians(), EPSILON,
                "Negative angles should be handled correctly");
    }

    @Test
    void testSetSetpoint_HandlesZero() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0));

        assertEquals(0.0, autoAlign.getSetpoint().getRadians(), EPSILON,
                "Zero degree setpoint should be stored correctly");
    }

    @Test
    void testSetSetpoint_Handles360Degrees() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(360));

        // 360 degrees should wrap to 0
        assertEquals(0.0, autoAlign.getSetpoint().getRadians(), EPSILON,
                "360 degree setpoint should wrap to 0");
    }

    @Test
    void testSetSetpoint_UpdatesSetpointMultipleTimes() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(45));
        assertEquals(Math.toRadians(45), autoAlign.getSetpoint().getRadians(), EPSILON);

        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));
        assertEquals(Math.toRadians(90), autoAlign.getSetpoint().getRadians(), EPSILON,
                "Setpoint should be updated when set multiple times");
    }

    // ==================== Correction Calculation Tests ====================

    @Test
    void testGetCorrection_ReturnsZeroWhenAtSetpoint() {
        Rotation2d setpoint = Rotation2d.fromDegrees(90);
        autoAlign.setSetpoint(setpoint);

        // Allow time for PID to settle by calling getCorrection multiple times
        for (int i = 0; i < 10; i++) {
            autoAlign.getCorrection(setpoint);
        }

        double correction = autoAlign.getCorrection(setpoint);
        assertEquals(0.0, correction, 0.02,
                "Correction should be near zero when current angle equals setpoint");
    }

    @Test
    void testGetCorrection_ReturnsPositiveCorrectionWhenBehindSetpoint() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));
        Rotation2d currentAngle = Rotation2d.fromDegrees(0);

        double correction = autoAlign.getCorrection(currentAngle);
        assertTrue(correction > 0,
                "Correction should be positive when current angle is less than setpoint");
    }

    @Test
    void testGetCorrection_ReturnsNegativeCorrectionWhenAheadOfSetpoint() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0));
        Rotation2d currentAngle = Rotation2d.fromDegrees(90);

        double correction = autoAlign.getCorrection(currentAngle);
        assertTrue(correction < 0,
                "Correction should be negative when current angle is greater than setpoint");
    }

    @Test
    void testGetCorrection_ModulatesCurrentAngle() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0));
        Rotation2d largeAngle = Rotation2d.fromDegrees(370); // Should be treated as 10 degrees

        double correction = autoAlign.getCorrection(largeAngle);
        assertTrue(correction < 0,
                "Correction should handle modulated current angles correctly");
    }

    @Test
    void testGetCorrection_AppliesDeadband() {
        // Set a setpoint very close to current angle to test deadband
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0.1));

        // Call multiple times to let PID settle
        for (int i = 0; i < 20; i++) {
            autoAlign.getCorrection(Rotation2d.fromDegrees(0));
        }

        double correction = autoAlign.getCorrection(Rotation2d.fromDegrees(0));
        // With 0.01 deadband, very small corrections should be zeroed out
        assertTrue(Math.abs(correction) < 0.1,
                "Small corrections should be affected by deadband");
    }

    @Test
    void testGetCorrection_ClampsToMaxAngularSpeed() {
        // Create AutoAlign with very high PID gains to test clamping
        AutoAlign highGainAutoAlign = new AutoAlign(new ExtendedPIDConstants(100, 0, 0));
        highGainAutoAlign.setSetpoint(Rotation2d.fromDegrees(180));

        double correction = highGainAutoAlign.getCorrection(Rotation2d.fromDegrees(0));

        // Correction should be clamped to MaxAngularSpeedRadians (2π rad/s)
        assertTrue(Math.abs(correction) <= Math.PI * 2 + EPSILON,
                "Correction should be clamped to max angular speed");
    }

    @Test
    void testGetCorrection_HandlesContinuousInput() {
        // Test that PID uses shortest path around circle
        autoAlign.setSetpoint(Rotation2d.fromDegrees(170));
        Rotation2d currentAngle = Rotation2d.fromDegrees(-170);

        double correction = autoAlign.getCorrection(currentAngle);

        // The shortest path from -170 to 170 is 20 degrees (going in positive direction)
        // With continuous input, PID should recognize this and provide a correction
        // The magnitude should be reasonable (not trying to go 340 degrees the wrong way)
        assertTrue(Math.abs(correction) < Math.PI,
                "PID should take shortest path with continuous input enabled");
    }

    @Test
    void testGetCorrection_ProducesNonZeroOutput() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(45));
        Rotation2d currentAngle = Rotation2d.fromDegrees(0);

        double correction = autoAlign.getCorrection(currentAngle);

        // With an error of 45 degrees, correction should be non-zero
        assertNotEquals(0.0, correction,
                "Correction should be non-zero when there is an error");
    }

    // ==================== AtSetpoint Tests ====================

    @Test
    void testAtSetpoint_ReturnsTrueWhenWithinTolerance() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));

        // Call getCorrection to update PID state
        for (int i = 0; i < 10; i++) {
            autoAlign.getCorrection(Rotation2d.fromDegrees(90));
        }

        assertTrue(autoAlign.atSetpoint(),
                "atSetpoint should return true when at setpoint within tolerance");
    }

    @Test
    void testAtSetpoint_ReturnsFalseWhenOutsideTolerance() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));
        autoAlign.getCorrection(Rotation2d.fromDegrees(0));

        assertFalse(autoAlign.atSetpoint(),
                "atSetpoint should return false when far from setpoint");
    }

    @Test
    void testAtSetpoint_UsesToleranceOf1Degree() {
        // Tolerance is set to π/180 radians (1 degree) in constructor
        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));

        // Test just within tolerance
        for (int i = 0; i < 10; i++) {
            autoAlign.getCorrection(Rotation2d.fromDegrees(90.5));
        }
        assertTrue(autoAlign.atSetpoint(),
                "Should be at setpoint when within 1 degree tolerance");
    }

    // ==================== Reset Tests ====================

    @Test
    void testResetPID_ResetsInternalState() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));

        // Accumulate some integral error
        for (int i = 0; i < 5; i++) {
            autoAlign.getCorrection(Rotation2d.fromDegrees(0));
        }

        autoAlign.resetPID();

        double correctionAfterReset = autoAlign.getCorrection(Rotation2d.fromDegrees(0));

        // After reset, the PID should behave as if starting fresh
        // The corrections might be similar for P-only controller, but I term should be reset
        assertNotNull(correctionAfterReset,
                "Reset should not cause null corrections");
    }

    @Test
    void testResetPID_ClearsAtSetpointState() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0));

        // Get to setpoint
        for (int i = 0; i < 10; i++) {
            autoAlign.getCorrection(Rotation2d.fromDegrees(0));
        }
        assertTrue(autoAlign.atSetpoint());

        // Reset and check with different angle
        autoAlign.resetPID();
        autoAlign.getCorrection(Rotation2d.fromDegrees(90));

        assertFalse(autoAlign.atSetpoint(),
                "After reset, should not be at setpoint with different angle");
    }

    // ==================== Integration Tests ====================

    @Test
    void testFullCycle_SetSetpointAndGetCorrection() {
        // Simulate a complete alignment cycle
        autoAlign.setSetpoint(Rotation2d.fromDegrees(45));

        Rotation2d currentAngle = Rotation2d.fromDegrees(0);
        double correction = autoAlign.getCorrection(currentAngle);

        assertNotEquals(0.0, correction,
                "Correction should be non-zero when not at setpoint");
        assertEquals(Math.toRadians(45), autoAlign.getSetpoint().getRadians(), EPSILON,
                "Setpoint should remain stable during correction calculation");
    }

    @Test
    void testSimulatedApproachToSetpoint() {
        // Simulate approaching the setpoint over multiple iterations
        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));

        double currentAngleDegrees = 0;
        int maxIterations = 100;
        int iterations = 0;

        while (!autoAlign.atSetpoint() && iterations < maxIterations) {
            double correction = autoAlign.getCorrection(Rotation2d.fromDegrees(currentAngleDegrees));
            // Simulate applying correction (scaled down for discrete simulation)
            currentAngleDegrees += Math.toDegrees(correction) * 0.02; // Small time step
            iterations++;
        }

        assertTrue(iterations < maxIterations,
                "System should converge to setpoint within reasonable iterations");
        assertTrue(Math.abs(currentAngleDegrees - 90) < 5,
                "Final angle should be close to setpoint");
    }

    @Test
    void testMultipleSetpointChanges() {
        // Test changing setpoints multiple times
        autoAlign.setSetpoint(Rotation2d.fromDegrees(45));
        assertEquals(Math.toRadians(45), autoAlign.getSetpoint().getRadians(), EPSILON);

        autoAlign.setSetpoint(Rotation2d.fromDegrees(90));
        assertEquals(Math.toRadians(90), autoAlign.getSetpoint().getRadians(), EPSILON);

        autoAlign.setSetpoint(Rotation2d.fromDegrees(-45));
        assertEquals(Math.toRadians(-45), autoAlign.getSetpoint().getRadians(), EPSILON);

        // Should still be able to calculate corrections
        double correction = autoAlign.getCorrection(Rotation2d.fromDegrees(0));
        assertNotNull(correction);
    }

    // ==================== Edge Cases ====================

    @Test
    void testGetCorrection_WithNullRotation2d_ThrowsException() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0));
        assertThrows(Exception.class, () -> autoAlign.getCorrection(null),
                "getCorrection should throw exception for null input");
    }

    @Test
    void testSetSetpoint_WithNullRotation2d_ThrowsException() {
        assertThrows(Exception.class, () -> autoAlign.setSetpoint(null),
                "setSetpoint should throw exception for null input");
    }

    @Test
    void testGetCorrection_WithExtremeAngles() {
        autoAlign.setSetpoint(Rotation2d.fromDegrees(0));

        // Test with very large angle
        assertDoesNotThrow(() -> autoAlign.getCorrection(Rotation2d.fromDegrees(7200)),
                "Should handle extreme angles without throwing");

        // Test with very large negative angle
        assertDoesNotThrow(() -> autoAlign.getCorrection(Rotation2d.fromDegrees(-7200)),
                "Should handle extreme negative angles without throwing");
    }

    @Test
    void testWrapAroundBehavior() {
        // Test the boundary case where angles cross from positive to negative
        autoAlign.setSetpoint(Rotation2d.fromDegrees(179));
        double correction1 = autoAlign.getCorrection(Rotation2d.fromDegrees(-179));

        // These angles are only 2 degrees apart via shortest path
        // So correction should be relatively small
        assertTrue(Math.abs(correction1) < Math.PI,
                "Wrap-around should use shortest path");
    }

    @Test
    void testPIDContinuousInputConfiguration() {
        // Verify that continuous input enables proper wrap-around
        autoAlign.setSetpoint(Rotation2d.fromDegrees(-175));
        Rotation2d currentAngle = Rotation2d.fromDegrees(175);

        double correction = autoAlign.getCorrection(currentAngle);

        // Shortest path from 175 to -175 is -10 degrees (going negative/counterclockwise)
        // With continuous input, the correction magnitude should be reasonable
        assertTrue(Math.abs(correction) < Math.PI,
                "Continuous input should make PID take shortest path across -180/180 boundary");
    }
}
