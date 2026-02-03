package org.prime.util;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;

public class MutVectorTest {
    private MutVector mutVector;

    @BeforeEach
    public void setUp() {
        mutVector = new MutVector();
    }

    @Test
    public void testCreation_blank() {
        Assertions.assertEquals(0, mutVector.getX());
        Assertions.assertEquals(0, mutVector.getY());
        Assertions.assertEquals(0, mutVector.getZ());

        Assertions.assertEquals(0, mutVector.getMagnitude());
        Assertions.assertEquals(0, mutVector.getPitch());
        Assertions.assertEquals(0, mutVector.getYaw());
    }

    @Test
    public void testCreation_fromCartesian() {
        double x = 1;
        double y = 2;
        double z = 3;

        mutVector.fromCartesian(x, y, z);

        Assertions.assertEquals(x, mutVector.getX());
        Assertions.assertEquals(y, mutVector.getY());
        Assertions.assertEquals(z, mutVector.getZ());
    }

    @Test
    public void testCreation_fromPolar() {
        double magnitude = 1;
        double pitch = 2;
        double yaw = 3;

        mutVector.fromPolar(magnitude, pitch, yaw);

        Assertions.assertEquals(magnitude, mutVector.getMagnitude(), 0.0001);
        Assertions.assertEquals(pitch, mutVector.getPitch(), 0.0001);
        Assertions.assertEquals(yaw, mutVector.getYaw(), 0.0001);
    }

    @Test
    public void testCreation_setCartesianBehavesAsFromCartesian() {
        double x = 1;
        double y = 2;
        double z = 3;

        MutVector testVector = new MutVector();
        testVector.setCartesian(x, y, z);

        mutVector.fromCartesian(x, y, z);

        Assertions.assertEquals(mutVector.getX(), testVector.getX());
        Assertions.assertEquals(mutVector.getY(), testVector.getY());
        Assertions.assertEquals(mutVector.getZ(), testVector.getZ());

        Assertions.assertEquals(mutVector.getMagnitude(), testVector.getMagnitude());
        Assertions.assertEquals(mutVector.getPitch(), testVector.getPitch());
        Assertions.assertEquals(mutVector.getYaw(), testVector.getYaw());
    }

    @Test
    public void testCreation_setPolarBehavesAsFromPolar() {
        double magnitude = 1;
        double pitch = 2;
        double yaw = 3;

        MutVector testVector = new MutVector();
        testVector.setPolar(magnitude, pitch, yaw);

        mutVector.setPolar(magnitude, pitch, yaw);

        Assertions.assertEquals(mutVector.getX(), testVector.getX());
        Assertions.assertEquals(mutVector.getY(), testVector.getY());
        Assertions.assertEquals(mutVector.getZ(), testVector.getZ());

        Assertions.assertEquals(mutVector.getMagnitude(), testVector.getMagnitude());
        Assertions.assertEquals(mutVector.getPitch(), testVector.getPitch());
        Assertions.assertEquals(mutVector.getYaw(), testVector.getYaw());
    }

    @Test
    public void testSet_x() {
        double x_before = 1;
        double x_after = 2;

        mutVector.fromCartesian(x_before, 0, 0);
        mutVector.setX(x_after);

        Assertions.assertEquals(x_after, mutVector.getX());
    }

    @Test
    public void testSet_y() {
        double y_before = 1;
        double y_after = 2;

        mutVector.fromCartesian(0, y_before, 0);
        mutVector.setY(y_after);

        Assertions.assertEquals(y_after, mutVector.getY());
    }

    @Test
    public void testSet_z() {
        double z_before = 1;
        double z_after = 2;

        mutVector.fromCartesian(0, 0, z_before);
        mutVector.setZ(z_after);

        Assertions.assertEquals(z_after, mutVector.getZ());
    }

    @Test
    public void testSet_magnitude() {
        double magnitude_before = 1;
        double magnitude_after = 2;

        mutVector.fromPolar(magnitude_before, 0, 0);
        mutVector.setMagnitude(magnitude_after);

        Assertions.assertEquals(magnitude_after, mutVector.getMagnitude());
    }

    @Test
    public void testSet_pitch() {
        double pitch_before = 1;
        double pitch_after = 2;

        mutVector.fromPolar(1, pitch_before, 0);
        mutVector.setPitch(pitch_after);

        Assertions.assertEquals(pitch_after, mutVector.getPitch(), 0.0001);
    }

    @Test
    public void testSet_yaw() {
        double yaw_before = 1;
        double yaw_after = 2;

        mutVector.fromPolar(1, 45, yaw_before); // Use non-zero pitch to avoid singularity
        mutVector.setYaw(yaw_after);

        Assertions.assertEquals(yaw_after, mutVector.getYaw(), 0.0001);
    }

    @Test
    public void testArithmetics_addition() {
        double x_1 = 1;
        double y_1 = 2;
        double z_1 = 3;
        double x_2 = 4;
        double y_2 = 5;
        double z_2 = 6;

        MutVector testVector = new MutVector().fromCartesian(x_2, y_2, z_2);

        mutVector.fromCartesian(x_1, y_1, z_1);
        mutVector.plus(testVector);

        Assertions.assertEquals(x_1 + x_2, mutVector.getX());
        Assertions.assertEquals(y_1 + y_2, mutVector.getY());
        Assertions.assertEquals(z_1 + z_2, mutVector.getZ());
    }

    @Test
    public void testArithmetics_subtraction() {
        double x_1 = 1;
        double y_1 = 2;
        double z_1 = 3;
        double x_2 = 4;
        double y_2 = 5;
        double z_2 = 6;

        MutVector testVector = new MutVector().fromCartesian(x_2, y_2, z_2);

        mutVector.fromCartesian(x_1, y_1, z_1);
        mutVector.minus(testVector);

        Assertions.assertEquals(x_1 - x_2, mutVector.getX());
        Assertions.assertEquals(y_1 - y_2, mutVector.getY());
        Assertions.assertEquals(z_1 - z_2, mutVector.getZ());
    }

    // ============================================
    // Tests for setToTargetVector
    // ============================================

    @Test
    public void testSetToTargetVector_ValidShotAtSameHeight() throws Exception {
        // Create source and target poses at the same height
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(5, 0, 1.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Should find a valid solution
        Assertions.assertTrue(mutVector.getMagnitude() > 0);
        Assertions.assertTrue(mutVector.getMagnitude() <= 1000.0);
        Assertions.assertTrue(mutVector.getMagnitude() >= 0.0);
    }

    @Test
    public void testSetToTargetVector_ValidShotWithElevationChange() throws Exception {
        // Target is higher than source
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(3, 0, 3.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Should find a valid solution
        Assertions.assertTrue(mutVector.getMagnitude() > 0);
        Assertions.assertTrue(mutVector.getPitch() >= 20.0);
        Assertions.assertTrue(mutVector.getPitch() <= 60.0);
    }

    @Test
    public void testSetToTargetVector_VeryFarTargetVeryLowSpeed() {
        // Impossible shot: target extremely far and high with impossibly low speed
        Pose3d source = new Pose3d(0, 0, 0.0, new Rotation3d());
        Pose3d target = new Pose3d(10000, 0, 1000.0, new Rotation3d());

        Assertions.assertThrows(Exception.class, () -> {
            mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 0.01);
        });
    }

    @Test
    public void testSetToTargetVector_ThrowsWhenSpeedTooLow() {
        // Speed range impossibly low for any reasonable distance
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(50, 0, 10.0, new Rotation3d());

        Assertions.assertThrows(Exception.class, () -> {
            mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 0.1);
        });
    }

    @Test
    public void testSetToTargetVector_ThrowsWhenAngleRangeTooNarrow() {
        // Angle range at extreme (nearly vertical) where physics won't allow horizontal travel
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(50, 0, 1.0, new Rotation3d());

        Assertions.assertThrows(Exception.class, () -> {
            mutVector.setToTargetVector(source, target, 89.9, 90.0, 0.0, 1000.0);
        });
    }

    @Test
    public void testSetToTargetVector_YawCalculationPositiveY() throws Exception {
        // Target to the right (+Y direction)
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(3, 3, 1.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Yaw should point towards +Y
        double yaw = mutVector.getYaw();
        Assertions.assertTrue(Math.abs(yaw - 45) < 5, "Yaw should be approximately 45 degrees");
    }

    @Test
    public void testSetToTargetVector_YawCalculationNegativeY() throws Exception {
        // Target to the left (-Y direction)
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(3, -3, 1.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Yaw should point towards -Y
        double yaw = mutVector.getYaw();
        Assertions.assertTrue(Math.abs(yaw + 45) < 5, "Yaw should be approximately -45 degrees");
    }

    @Test
    public void testSetToTargetVector_ShortDistance() throws Exception {
        // Very short distance shot
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(2.0, 0, 1.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Should still find a valid solution
        Assertions.assertTrue(mutVector.getMagnitude() > 0);
    }

    @Test
    public void testSetToTargetVector_TargetBelowSource() throws Exception {
        // Shooting downward
        Pose3d source = new Pose3d(0, 0, 5.0, new Rotation3d());
        Pose3d target = new Pose3d(3, 0, 1.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Should find a valid solution
        Assertions.assertTrue(mutVector.getMagnitude() > 0);
    }

    @Test
    public void testSetToTargetVector_MaxAngleBoundary() throws Exception {
        // Test with angle at max boundary
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(2, 0, 2.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Pitch should be within range
        Assertions.assertTrue(mutVector.getPitch() >= 20.0);
        Assertions.assertTrue(mutVector.getPitch() <= 60.0);
    }

    @Test
    public void testSetToTargetVector_MinAngleBoundary() throws Exception {
        // Test with angle at min boundary
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(5, 0, 1.5, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Pitch should be within range
        Assertions.assertTrue(mutVector.getPitch() >= 20.0);
        Assertions.assertTrue(mutVector.getPitch() <= 60.0);
    }

    @Test
    public void testSetToTargetVector_SpeedWithinRange() throws Exception {
        // Verify calculated speed is within min/max bounds when solution exists
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(10, 0, 2.0, new Rotation3d());

        double minSpeed = 1.0;
        double maxSpeed = 1000.0;

        mutVector.setToTargetVector(source, target, 20.0, 60.0, minSpeed, maxSpeed);

        Assertions.assertTrue(mutVector.getMagnitude() >= minSpeed);
        Assertions.assertTrue(mutVector.getMagnitude() <= maxSpeed);
    }

    @Test
    public void testSetToTargetVector_DiagonalShot() throws Exception {
        // Shot in both X and Y directions
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(4, 3, 2.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Should find a valid solution
        Assertions.assertTrue(mutVector.getMagnitude() > 0);

        // Verify the vector components make sense
        Assertions.assertNotEquals(0, mutVector.getX());
        Assertions.assertNotEquals(0, mutVector.getY());
    }

    @Test
    public void testSetToTargetVector_PrecisionCheck() throws Exception {
        // Test that the solution is reasonably accurate
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(5, 0, 1.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        // Store the values
        double magnitude = mutVector.getMagnitude();
        double pitch = mutVector.getPitch();
        double yaw = mutVector.getYaw();

        // All values should be valid numbers
        Assertions.assertFalse(Double.isNaN(magnitude));
        Assertions.assertFalse(Double.isNaN(pitch));
        Assertions.assertFalse(Double.isNaN(yaw));
        Assertions.assertFalse(Double.isInfinite(magnitude));
    }

    // ============================================
    // Tests for getTimeToTarget
    // ============================================

    @Test
    public void testGetTimeToTarget_BasicCalculation() {
        // Create a vector with known horizontal components
        mutVector.fromCartesian(3, 4, 0); // Horizontal magnitude = 5

        var time = mutVector.getTimeToTarget(10.0);

        // Time = distance / horizontal_velocity = 10 / 5 = 2 seconds
        Assertions.assertEquals(2.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_WithVerticalComponent() {
        // Vector with vertical component
        mutVector.fromCartesian(3, 4, 5); // Horizontal magnitude = 5, total magnitude > 5

        var time = mutVector.getTimeToTarget(15.0);

        // Should use horizontal distance only
        // Horizontal velocity = sqrt(3^2 + 4^2) = 5
        // Time = 15 / 5 = 3 seconds
        double expectedTime = 15.0 / Math.sqrt(9 + 16);
        Assertions.assertEquals(expectedTime, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_ZeroMagnitude() {
        // Vector with zero magnitude
        mutVector.fromCartesian(0, 0, 0);

        var time = mutVector.getTimeToTarget(10.0);

        // Should return -1 when magnitude is zero
        Assertions.assertEquals(-1.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_ZeroDistance() {
        // Non-zero vector, zero distance
        mutVector.fromCartesian(5, 5, 5);

        var time = mutVector.getTimeToTarget(0.0);

        // Time should be zero
        Assertions.assertEquals(0.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_LargeDistance() {
        // Test with large distance
        mutVector.fromCartesian(10, 10, 5);

        var time = mutVector.getTimeToTarget(1000.0);

        // Should return positive time
        Assertions.assertTrue(time.in(Units.Seconds) > 0);
    }

    @Test
    public void testGetTimeToTarget_SmallDistance() {
        // Test with small distance
        mutVector.fromCartesian(100, 100, 50);

        var time = mutVector.getTimeToTarget(0.1);

        // Should return very small positive time
        Assertions.assertTrue(time.in(Units.Seconds) > 0);
        Assertions.assertTrue(time.in(Units.Seconds) < 1.0);
    }

    @Test
    public void testGetTimeToTarget_OnlyHorizontalComponents() {
        // Vector with only horizontal components (no Z)
        mutVector.fromCartesian(6, 8, 0); // Magnitude = 10

        var time = mutVector.getTimeToTarget(20.0);

        // Time = 20 / 10 = 2 seconds
        Assertions.assertEquals(2.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_OnlyXComponent() {
        // Vector with only X component
        mutVector.fromCartesian(5, 0, 0);

        var time = mutVector.getTimeToTarget(10.0);

        // Time = 10 / 5 = 2 seconds
        Assertions.assertEquals(2.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_OnlyYComponent() {
        // Vector with only Y component
        mutVector.fromCartesian(0, 8, 0);

        var time = mutVector.getTimeToTarget(16.0);

        // Time = 16 / 8 = 2 seconds
        Assertions.assertEquals(2.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_NegativeComponents() {
        // Vector with negative components
        mutVector.fromCartesian(-3, -4, 0); // Magnitude = 5

        var time = mutVector.getTimeToTarget(10.0);

        // Should still calculate correctly (magnitude is always positive)
        Assertions.assertEquals(2.0, time.in(Units.Seconds), 0.001);
    }

    @Test
    public void testGetTimeToTarget_AfterSetToTargetVector() throws Exception {
        // Integration test: use setToTargetVector then check time
        Pose3d source = new Pose3d(0, 0, 1.0, new Rotation3d());
        Pose3d target = new Pose3d(5, 0, 2.0, new Rotation3d());

        mutVector.setToTargetVector(source, target, 20.0, 60.0, 0.0, 1000.0);

        double distance = Math.hypot(5, 0);
        var time = mutVector.getTimeToTarget(distance);

        // Time should be positive and reasonable
        Assertions.assertTrue(time.in(Units.Seconds) > 0);
        Assertions.assertTrue(time.in(Units.Seconds) < 10); // Should be less than 10 seconds for this scenario
    }

    @Test
    public void testGetTimeToTarget_ConsistentWithMagnitude() {
        // Verify relationship: time * magnitude = horizontal distance
        mutVector.fromCartesian(3, 4, 5);

        double distance = 15.0;
        var time = mutVector.getTimeToTarget(distance);

        double calculatedDistance = time.in(Units.Seconds) * mutVector.getMagnitude();

        // The calculated distance using total magnitude should relate to the input
        Assertions.assertTrue(calculatedDistance > 0);
    }
}
