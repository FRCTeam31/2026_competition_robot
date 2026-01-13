package frc.robot.subsystems.swerve.gyro;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

/**
 * Unit tests for GyroSim - the simulated gyroscope implementation.
 * Tests focus on real robot scenarios:
 * - Robot rotation during autonomous and teleop
 * - Field-oriented drive angle tracking
 * - Gyro reset during enable/disable
 * - Continuous rotation accumulation
 */
class GyroSimTest {
    private GyroSim gyro;
    private GyroInputsAutoLogged inputs;

    // Standard robot loop period (20ms)
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        gyro = new GyroSim(0);
        inputs = new GyroInputsAutoLogged();

        // Reset gyro to known state before each test
        gyro.reset();
        gyro.updateInputs(inputs, 0);
    }

    //#region Initialization Tests

    @Test
    void testGyroStartsAtZero() {
        gyro.updateInputs(inputs, 0);

        assertEquals(0.0, inputs.Rotation.getDegrees(), 0.001,
                "Gyro should initialize at 0 degrees");
    }

    @Test
    void testAccelerationsInitializeToZero() {
        gyro.updateInputs(inputs, 0);

        assertEquals(0.0, inputs.AccelerationX, 0.001, "X acceleration should be 0");
        assertEquals(0.0, inputs.AccelerationY, 0.001, "Y acceleration should be 0");
        assertEquals(0.0, inputs.AccelerationZ, 0.001, "Z acceleration should be 0");
    }

    //#endregion

    //#region Real Robot Scenario: Stationary Robot

    @Test
    void testStationaryRobotMaintainsHeading() {
        // Robot sitting still - no rotation
        for (int i = 0; i < 50; i++) {
            gyro.updateInputs(inputs, 0.0);
        }

        assertEquals(0.0, inputs.Rotation.getDegrees(), 0.001,
                "Stationary robot should maintain 0 degree heading");
    }

    //#endregion

    //#region Real Robot Scenario: Constant Rotation Speed

    @Test
    void testConstantClockwiseRotation() {
        // Robot rotating clockwise at 90 degrees per second (common max for swerve)
        double omegaRadPerSec = -Math.toRadians(90); // Negative = clockwise

        // Simulate 1 second of rotation (50 loops at 20ms each)
        for (int i = 0; i < 50; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(-90.0, inputs.Rotation.getDegrees(), 0.1,
                "After 1 second at 90-degrees/s clockwise, heading should be -90-degrees");
    }

    @Test
    void testConstantCounterClockwiseRotation() {
        // Robot rotating counter-clockwise at 180 degrees per second
        double omegaRadPerSec = Math.toRadians(180); // Positive = counter-clockwise

        // Simulate 0.5 seconds of rotation (25 loops)
        for (int i = 0; i < 25; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(90.0, inputs.Rotation.getDegrees(), 0.1,
                "After 0.5 seconds at 180-degrees/s CCW, heading should be 90-degrees");
    }

    @Test
    void testSlowRotation() {
        // Robot slowly drifting/rotating at 5 degrees per second
        double omegaRadPerSec = Math.toRadians(5);

        // Simulate 2 seconds of slow rotation (100 loops)
        for (int i = 0; i < 100; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(10.0, inputs.Rotation.getDegrees(), 0.1,
                "After 2 seconds at 5-degrees/s, heading should be 10-degrees");
    }

    //#endregion

    //#region Real Robot Scenario: Autonomous Rotation

    @Test
    void testAutonomous90DegreeTurn() {
        // Simulate a PID-controlled 90-degree turn in autonomous
        // In practice, PID would vary the speed - simplified here to constant speed
        double omegaRadPerSec = Math.toRadians(180); // 180-degrees/s average speed

        // Rotate for 0.5 seconds to achieve ~90-degrees
        for (int i = 0; i < 25; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        // Should be close to 90 degrees
        assertEquals(90.0, inputs.Rotation.getDegrees(), 2.0,
                "Autonomous 90-degrees turn should result in heading near 90-degrees");
    }

    @Test
    void testAutonomous180DegreeTurn() {
        // Fast 180-degree spin at 360-degrees/s for 0.5 seconds
        double omegaRadPerSec = Math.toRadians(360);

        for (int i = 0; i < 25; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(180.0, inputs.Rotation.getDegrees(), 1.0,
                "180-degrees turn should result in heading of 180-degrees");
    }

    //#endregion

    //#region Real Robot Scenario: Full Rotation and Wraparound

    @Test
    void testFullClockwiseRotation() {
        // Complete 360-degree clockwise rotation
        double omegaRadPerSec = -Math.toRadians(360);

        // 1 second rotation
        for (int i = 0; i < 50; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        // Gyro should handle full rotation (might wrap to 0 or stay at -360)
        double heading = inputs.Rotation.getDegrees();
        assertTrue(Math.abs(heading) < 1.0 || Math.abs(heading + 360) < 1.0,
                "After 360-degrees clockwise rotation, heading should be near 0-degrees or -360-degrees, got: "
                        + heading);
    }

    @Test
    void testMultipleRotations() {
        // Robot spinning continuously (like testing rotation in place)
        // 720 degrees in 2 seconds = 360-degrees/s
        double omegaRadPerSec = Math.toRadians(360);

        for (int i = 0; i < 100; i++) { // 2 seconds
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        // After 2 full rotations, could be 720-degrees or wrapped
        double heading = inputs.Rotation.getDegrees();
        assertTrue(Math.abs(heading) < 1.0 || Math.abs(heading - 720) < 1.0,
                "After 720-degrees rotation, heading handling should be consistent");
    }

    //#endregion

    //#region Real Robot Scenario: Direction Changes

    @Test
    void testOscillatingRotation() {
        // Robot rotating back and forth (like driver correcting heading)

        // Rotate 45-degrees CCW
        double omegaCCW = Math.toRadians(180);
        for (int i = 0; i < 12; i++) { // 0.24 seconds
            gyro.updateInputs(inputs, omegaCCW);
        }

        double headingAfterCCW = inputs.Rotation.getDegrees();
        assertTrue(headingAfterCCW > 40.0 && headingAfterCCW < 50.0,
                "After CCW rotation should be ~45-degrees, got: " + headingAfterCCW);

        // Rotate back 45-degrees CW
        double omegaCW = -Math.toRadians(180);
        for (int i = 0; i < 12; i++) {
            gyro.updateInputs(inputs, omegaCW);
        }

        assertEquals(0.0, inputs.Rotation.getDegrees(), 2.0,
                "After oscillation, should return near 0-degrees");
    }

    @Test
    void testRapidDirectionChanges() {
        // Simulate joystick wiggling - rapid small rotations
        for (int i = 0; i < 20; i++) {
            double omega = (i % 2 == 0) ? Math.toRadians(45) : -Math.toRadians(45);
            gyro.updateInputs(inputs, omega);
        }

        // Should end up close to starting position
        assertTrue(Math.abs(inputs.Rotation.getDegrees()) < 5.0,
                "Rapid alternating rotations should mostly cancel out");
    }

    //#endregion

    //#region Real Robot Scenario: Gyro Reset

    @Test
    void testResetDuringMatch() {
        // Robot has rotated, then reset (like at start of autonomous)
        double omegaRadPerSec = Math.toRadians(90);

        // Rotate 45 degrees
        for (int i = 0; i < 25; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertNotEquals(0.0, inputs.Rotation.getDegrees(), 0.1,
                "Before reset, gyro should not be at 0");

        // Reset gyro
        gyro.reset();

        // Update inputs to get new value
        gyro.updateInputs(inputs, 0);

        assertEquals(0.0, inputs.Rotation.getDegrees(), 0.001,
                "After reset, gyro should be at 0-degrees");
    }

    @Test
    void testResetToSpecificAngle() {
        // Reset gyro to field-oriented angle (like when starting at known position)
        gyro.reset(180.0);

        gyro.updateInputs(inputs, 0);

        assertEquals(180.0, inputs.Rotation.getDegrees(), 0.001,
                "After reset to 180-degrees, gyro should read 180-degrees");
    }

    @Test
    void testResetToNegativeAngle() {
        // Reset to negative angle (like -90-degrees starting position)
        gyro.reset(-90.0);

        gyro.updateInputs(inputs, 0);

        assertEquals(-90.0, inputs.Rotation.getDegrees(), 0.001,
                "After reset to -90-degrees, gyro should read -90-degrees");
    }

    @Test
    void testRotationAfterReset() {
        // Verify rotation works correctly after reset
        gyro.reset(45.0);
        gyro.updateInputs(inputs, 0);

        // Now rotate 90 degrees CCW
        double omegaRadPerSec = Math.toRadians(180);
        for (int i = 0; i < 25; i++) { // 0.5 seconds
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(135.0, inputs.Rotation.getDegrees(), 1.0,
                "After reset to 45-degrees then 90-degrees rotation, should be at 135-degrees");
    }

    @Test
    void testMultipleResets() {
        // Simulate multiple enable/disable cycles with resets

        // First cycle
        gyro.reset();
        for (int i = 0; i < 10; i++) {
            gyro.updateInputs(inputs, Math.toRadians(90));
        }

        // Second cycle - reset and rotate opposite direction
        gyro.reset();
        gyro.updateInputs(inputs, 0);
        assertEquals(0.0, inputs.Rotation.getDegrees(), 0.001,
                "Second reset should return to 0-degrees");

        for (int i = 0; i < 10; i++) {
            gyro.updateInputs(inputs, -Math.toRadians(90));
        }

        assertTrue(inputs.Rotation.getDegrees() < 0,
                "After second cycle, should have negative rotation");
    }

    //#endregion

    //#region Real Robot Scenario: Field-Oriented Drive

    @Test
    void testFieldOrientedDriveScenario() {
        // Simulate field-oriented drive: robot rotates while driving
        // Start at 0-degrees, rotate to 90-degrees over 1 second while driving

        double rotationSpeed = Math.toRadians(90); // 90-degrees/s

        for (int i = 0; i < 50; i++) {
            gyro.updateInputs(inputs, rotationSpeed);

            // In real robot, translation would be adjusted based on gyro angle
            // Verify gyro is providing continuous updates
            assertTrue(inputs.Rotation.getDegrees() >= 0,
                    "Heading should be continuously increasing");
        }

        assertEquals(90.0, inputs.Rotation.getDegrees(), 0.5,
                "Field-oriented rotation should reach target angle");
    }

    //#endregion

    //#region Real Robot Scenario: Precision and Accuracy

    @Test
    void testSmallAngleAccuracy() {
        // Test precision for small rotations (like holding heading)
        double omegaRadPerSec = Math.toRadians(1); // Very slow 1-degrees/s

        for (int i = 0; i < 50; i++) { // 1 second
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(1.0, inputs.Rotation.getDegrees(), 0.1,
                "Small angle accumulation should be accurate to 0.1-degrees");
    }

    @Test
    void testHighSpeedAccuracy() {
        // Test at maximum expected rotation speed (540-degrees/s for some swerve drives)
        double omegaRadPerSec = Math.toRadians(540);

        // Rotate for 1/3 second to get 180-degrees
        for (int i = 0; i < 17; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(180.0, inputs.Rotation.getDegrees(), 5.0,
                "High-speed rotation should maintain accuracy within 5-degrees");
    }

    @Test
    void testZeroOmegaMaintainsAngle() {
        // Robot rotates then stops - gyro should hold angle

        // Rotate to 60-degrees
        double omegaRadPerSec = Math.toRadians(180);
        for (int i = 0; i < 17; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        double angleAfterRotation = inputs.Rotation.getDegrees();

        // Stop rotating - angle should remain constant
        for (int i = 0; i < 50; i++) {
            gyro.updateInputs(inputs, 0.0);
        }

        assertEquals(angleAfterRotation, inputs.Rotation.getDegrees(), 0.001,
                "Gyro should maintain angle when omega is 0");
    }

    //#endregion

    //#region Real Robot Scenario: Edge Cases

    @Test
    void testVerySmallOmega() {
        // Test with very small rotation speed (drift compensation scenario)
        double omegaRadPerSec = Math.toRadians(0.1);

        for (int i = 0; i < 500; i++) { // 10 seconds
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(1.0, inputs.Rotation.getDegrees(), 0.2,
                "Very small omega should accumulate correctly over time");
    }

    @Test
    void testNegativeToPositiveTransition() {
        // Start with negative rotation, cross through 0 to positive

        // Start at -45-degrees
        gyro.reset(-45.0);
        gyro.updateInputs(inputs, 0);

        // Rotate 90-degrees CCW to end at +45-degrees
        double omegaRadPerSec = Math.toRadians(180);
        for (int i = 0; i < 25; i++) {
            gyro.updateInputs(inputs, omegaRadPerSec);
        }

        assertEquals(45.0, inputs.Rotation.getDegrees(), 1.0,
                "Crossing through 0-degrees should work correctly");
    }

    @Test
    void testContinuousUpdatesSameOmega() {
        // Verify each update increments correctly with same omega
        double omegaRadPerSec = Math.toRadians(100);
        double expectedIncrement = Math.toDegrees(omegaRadPerSec * LOOP_PERIOD_SECONDS);

        gyro.updateInputs(inputs, omegaRadPerSec);
        double firstAngle = inputs.Rotation.getDegrees();

        gyro.updateInputs(inputs, omegaRadPerSec);
        double secondAngle = inputs.Rotation.getDegrees();

        assertEquals(expectedIncrement, secondAngle - firstAngle, 0.01,
                "Each update should increment by expected amount");
    }

    //#endregion

    //#region Accelerometer Tests (Simulated)

    @Test
    void testAccelerationsRemainZeroInSimulation() {
        // GyroSim doesn't simulate accelerations - they should always be 0

        // Update with various rotation speeds
        gyro.updateInputs(inputs, Math.toRadians(90));
        assertEquals(0.0, inputs.AccelerationX, 0.001);
        assertEquals(0.0, inputs.AccelerationY, 0.001);
        assertEquals(0.0, inputs.AccelerationZ, 0.001);

        gyro.updateInputs(inputs, -Math.toRadians(180));
        assertEquals(0.0, inputs.AccelerationX, 0.001);
        assertEquals(0.0, inputs.AccelerationY, 0.001);
        assertEquals(0.0, inputs.AccelerationZ, 0.001);

        gyro.updateInputs(inputs, 0.0);
        assertEquals(0.0, inputs.AccelerationX, 0.001);
        assertEquals(0.0, inputs.AccelerationY, 0.001);
        assertEquals(0.0, inputs.AccelerationZ, 0.001);
    }

    //#endregion
}
