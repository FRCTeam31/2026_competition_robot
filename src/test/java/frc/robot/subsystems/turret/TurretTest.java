package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.SuperStructure;

/**
 * Unit tests for the Turret subsystem.
 * Focuses on testing the getLimelightPose3dFromRobotCenter() function
 * to verify correct 3D pose calculation.
 */
class TurretTest {
    private Turret turret;
    private static final double EPSILON = 1e-6; // Tolerance for floating-point comparisons

    // Store original values to restore after tests
    private double originalLimelightOffsetX;
    private double originalLimelightOffsetY;
    private double originalLimelightOffsetZ;
    private double originalLimelightPitch;
    private double originalLimelightYaw;
    private double originalLimelightRoll;
    private Translation3d originalTurretRobotOrigin;

    @BeforeEach
    void setUp() {
        // Initialize HAL for WPILib
        assert HAL.initialize(500, 0);

        // Store original TurretMap values
        originalLimelightOffsetX = TurretMap.LIMELIGHT_OFFSET_X;
        originalLimelightOffsetY = TurretMap.LIMELIGHT_OFFSET_Y;
        originalLimelightOffsetZ = TurretMap.LIMELIGHT_OFFSET_Z;
        originalLimelightPitch = TurretMap.LIMELIGHT_PITCH;
        originalLimelightYaw = TurretMap.LIMELIGHT_YAW;
        originalLimelightRoll = TurretMap.LIMELIGHT_ROLL;
        originalTurretRobotOrigin = TurretMap.TURRET_ROBOT_ORIGIN;
        // Create turret in simulation mode
        turret = new Turret();
    }

    @AfterEach
    void tearDown() {
        // Restore original TurretMap values
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", originalLimelightOffsetX);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", originalLimelightOffsetY);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", originalLimelightOffsetZ);
        setStaticField(TurretMap.class, "LIMELIGHT_PITCH", originalLimelightPitch);
        setStaticField(TurretMap.class, "LIMELIGHT_YAW", originalLimelightYaw);
        setStaticField(TurretMap.class, "LIMELIGHT_ROLL", originalLimelightRoll);
        setStaticField(TurretMap.class, "TURRET_ROBOT_ORIGIN", originalTurretRobotOrigin);
    }

    //#region Helper Methods

    /**
     * Helper method to set static fields for testing
     */
    private void setStaticField(Class<?> clazz, String fieldName, Object value) {
        try {
            Field field = clazz.getDeclaredField(fieldName);
            field.setAccessible(true);
            field.set(null, value);
        } catch (Exception e) {
            throw new RuntimeException("Failed to set field " + fieldName + ": " + e.getMessage(), e);
        }
    }

    /**
     * Sets the turret rotation for testing
     */
    private void setTurretRotation(double radians) {
        SuperStructure.Turret.TurretRotation = new Rotation2d(radians);
    }

    //#endregion

    //#region Zero Configuration Tests

    @Test
    void testGetLimelightPose_AllZeroConfiguration() {
        // Set turret rotation to zero
        setTurretRotation(0.0);

        // With all offsets at zero, limelight should be at origin
        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(0.0, result.getX(), EPSILON, "X position should be 0");
        assertEquals(0.0, result.getY(), EPSILON, "Y position should be 0");
        assertEquals(0.0, result.getZ(), EPSILON, "Z position should be 0");
        assertEquals(0.0, result.getRotation().getX(), EPSILON, "Roll should be 0");
        assertEquals(0.0, result.getRotation().getY(), EPSILON, "Pitch should be 0");
        assertEquals(0.0, result.getRotation().getZ(), EPSILON, "Yaw should be 0");
    }

    @Test
    void testGetLimelightPose_ZeroTurretRotation_WithOffsets() {
        // Set limelight offset from turret center
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.2); // 20cm forward
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.1); // 10cm right
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.15); // 15cm up

        // Set turret center offset from robot center
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_X", 0.3); // 30cm forward
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Y", -0.05); // 5cm left
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Z", 0.5); // 50cm up

        // Set turret rotation to zero
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // At zero rotation, offsets should just add
        assertEquals(0.5, result.getX(), EPSILON, "X = 0.3 + 0.2");
        assertEquals(0.05, result.getY(), EPSILON, "Y = -0.05 + 0.1");
        assertEquals(0.65, result.getZ(), EPSILON, "Z = 0.5 + 0.15");
    }

    //#endregion

    //#region Turret Rotation Tests

    @Test
    void testGetLimelightPose_90DegreeRotation() {
        // Set limelight offset from turret center
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.2); // 20cm forward
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.0);

        // No turret center offset for simplicity
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_X", 0.0);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Y", 0.0);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Z", 0.0);

        // Rotate turret 90 degrees counterclockwise
        setTurretRotation(Math.PI / 2);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // After 90 degree rotation, forward (X) becomes left (Y)
        assertEquals(0.0, result.getX(), EPSILON, "X should be ~0");
        assertEquals(0.2, result.getY(), EPSILON, "Y should be 0.2 (rotated from X)");
        assertEquals(0.0, result.getZ(), EPSILON, "Z should be 0");

        // Rotation should include turret rotation
        assertEquals(Math.PI / 2, result.getRotation().getZ(), EPSILON, "Yaw should be PI/2");
    }

    @Test
    void testGetLimelightPose_180DegreeRotation() {
        // Set limelight offset from turret center
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.2);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.1);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.0);

        setTurretRotation(Math.PI); // 180 degrees

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // After 180 degree rotation, X becomes -X, Y becomes -Y
        assertEquals(-0.2, result.getX(), EPSILON, "X should be inverted");
        assertEquals(-0.1, result.getY(), EPSILON, "Y should be inverted");
        assertEquals(0.0, result.getZ(), EPSILON, "Z should be unchanged");

        assertEquals(Math.PI, result.getRotation().getZ(), EPSILON, "Yaw should be PI");
    }

    @Test
    void testGetLimelightPose_NegativeRotation() {
        // Set limelight offset
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.1);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.0);

        // Rotate turret -90 degrees (clockwise)
        setTurretRotation(-Math.PI / 2);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // After -90 degree rotation, forward (X) becomes right (-Y)
        assertEquals(0.0, result.getX(), EPSILON, "X should be ~0");
        assertEquals(-0.1, result.getY(), EPSILON, "Y should be -0.1");

        assertEquals(-Math.PI / 2, result.getRotation().getZ(), EPSILON, "Yaw should be -PI/2");
    }

    @Test
    void testGetLimelightPose_45DegreeRotation() {
        // Set limelight offset
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 1.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.0);

        // Rotate turret 45 degrees
        setTurretRotation(Math.PI / 4);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // After 45 degree rotation, should be at equal X and Y
        double expected = Math.sqrt(2) / 2; // cos(45) = sin(45) = sqrt(2)/2
        assertEquals(expected, result.getX(), EPSILON, "X should be cos(45)");
        assertEquals(expected, result.getY(), EPSILON, "Y should be sin(45)");
    }

    //#endregion

    //#region Limelight Fixed Rotation Tests

    @Test
    void testGetLimelightPose_WithFixedPitch() {
        setStaticField(TurretMap.class, "LIMELIGHT_PITCH", Math.PI / 6); // 30 degrees
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(Math.PI / 6, result.getRotation().getY(), EPSILON, "Pitch should be PI/6");
    }

    @Test
    void testGetLimelightPose_WithFixedYaw() {
        setStaticField(TurretMap.class, "LIMELIGHT_YAW", Math.PI / 4); // 45 degrees fixed offset
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(Math.PI / 4, result.getRotation().getZ(), EPSILON, "Yaw should be PI/4");
    }

    @Test
    void testGetLimelightPose_WithFixedRoll() {
        setStaticField(TurretMap.class, "LIMELIGHT_ROLL", Math.PI / 8); // 22.5 degrees
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(Math.PI / 8, result.getRotation().getX(), EPSILON, "Roll should be PI/8");
    }

    @Test
    void testGetLimelightPose_CombinedFixedYawAndTurretRotation() {
        setStaticField(TurretMap.class, "LIMELIGHT_YAW", Math.PI / 6); // 30 degrees fixed
        setTurretRotation(Math.PI / 3); // 60 degrees turret rotation

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Combined yaw should be sum of fixed yaw and turret rotation
        assertEquals(Math.PI / 2, result.getRotation().getZ(), EPSILON, "Yaw should be PI/6 + PI/3 = PI/2");
    }

    //#endregion

    //#region Complex Offset Tests

    @Test
    void testGetLimelightPose_ComplexConfiguration() {
        // Set complex offsets
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.25);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", -0.15);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.30);

        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_X", 0.10);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Y", 0.05);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Z", 0.60);

        setStaticField(TurretMap.class, "LIMELIGHT_PITCH", Math.toRadians(25));
        setStaticField(TurretMap.class, "LIMELIGHT_YAW", Math.toRadians(5));
        setStaticField(TurretMap.class, "LIMELIGHT_ROLL", Math.toRadians(0));

        setTurretRotation(Math.toRadians(135)); // 135 degrees

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Verify result is not null and has reasonable values
        assertNotNull(result);
        assertTrue(Math.abs(result.getX()) < 10, "X should be within reasonable bounds");
        assertTrue(Math.abs(result.getY()) < 10, "Y should be within reasonable bounds");
        assertEquals(0.90, result.getZ(), EPSILON, "Z should be sum of Z offsets");

        // Verify rotation components
        assertEquals(Math.toRadians(0), result.getRotation().getX(), EPSILON);
        assertEquals(Math.toRadians(25), result.getRotation().getY(), EPSILON);
        assertEquals(Math.toRadians(140), result.getRotation().getZ(), EPSILON, "Yaw = 135 + 5");
    }

    @Test
    void testGetLimelightPose_OnlyYOffsetWithRotation() {
        // Test with only Y offset (right/left)
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.2); // 20cm right
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.0);

        setTurretRotation(Math.PI / 2); // 90 degrees

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Y offset rotates 90 degrees: Y becomes -X
        assertEquals(-0.2, result.getX(), EPSILON, "Y offset rotated to -X");
        assertEquals(0.0, result.getY(), EPSILON, "Y should be ~0");
    }

    //#endregion

    //#region Edge Cases and Negative Tests

    @Test
    void testGetLimelightPose_VerySmallRotation() {
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 1.0);
        setTurretRotation(0.0001); // Very small rotation

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Should be very close to no rotation case
        assertEquals(1.0, result.getX(), 0.01, "X should be ~1.0");
        assertEquals(0.0, result.getY(), 0.01, "Y should be ~0");
    }

    @Test
    void testGetLimelightPose_FullRotation() {
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.5);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.3);

        setTurretRotation(2 * Math.PI); // Full 360 degree rotation

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // After full rotation, should be back to original position
        assertEquals(0.5, result.getX(), EPSILON, "X should match original offset");
        assertEquals(0.3, result.getY(), EPSILON, "Y should match original offset");
    }

    @Test
    void testGetLimelightPose_NegativeOffsets() {
        // Test with all negative offsets
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", -0.1);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", -0.2);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", -0.05);

        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_X", -0.15);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Y", -0.1);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Z", -0.2);

        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(-0.25, result.getX(), EPSILON, "X should be sum of negative offsets");
        assertEquals(-0.3, result.getY(), EPSILON, "Y should be sum of negative offsets");
        assertEquals(-0.25, result.getZ(), EPSILON, "Z should be sum of negative offsets");
    }

    @Test
    void testGetLimelightPose_LargeOffsets() {
        // Test with unrealistically large offsets to verify no overflow
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 10.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 5.0);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 3.0);

        setTurretRotation(Math.PI / 4);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertNotNull(result);
        assertTrue(Double.isFinite(result.getX()), "X should be finite");
        assertTrue(Double.isFinite(result.getY()), "Y should be finite");
        assertTrue(Double.isFinite(result.getZ()), "Z should be finite");
    }

    @Test
    void testGetLimelightPose_MultipleConsecutiveCalls() {
        // Verify function is idempotent (same result on multiple calls)
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.2);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.1);
        setTurretRotation(Math.PI / 3);

        Pose3d result1 = turret.getLimelightPose3dFromRobotCenter();
        Pose3d result2 = turret.getLimelightPose3dFromRobotCenter();
        Pose3d result3 = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(result1.getX(), result2.getX(), EPSILON, "Multiple calls should give same X");
        assertEquals(result1.getY(), result2.getY(), EPSILON, "Multiple calls should give same Y");
        assertEquals(result1.getZ(), result2.getZ(), EPSILON, "Multiple calls should give same Z");
        assertEquals(result1.getX(), result3.getX(), EPSILON, "Multiple calls should give same X");
    }

    @Test
    void testGetLimelightPose_RotationSymmetry() {
        // Test that rotating by -angle gives mirrored Y result
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.5);
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Y", 0.0);

        setTurretRotation(Math.PI / 6);
        Pose3d resultPositive = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(-Math.PI / 6);
        Pose3d resultNegative = turret.getLimelightPose3dFromRobotCenter();

        // X should be the same, Y should be opposite
        assertEquals(resultPositive.getX(), resultNegative.getX(), EPSILON, "X should be symmetric");
        assertEquals(resultPositive.getY(), -resultNegative.getY(), EPSILON, "Y should be opposite");
    }

    @Test
    void testGetLimelightPose_ZOffsetIndependentOfRotation() {
        // Z offset should not be affected by turret rotation
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_Z", 0.75);
        setStaticField(TurretMap.class, "TURRET_CENTER_OFFSET_Z", 0.50);

        setTurretRotation(0.0);
        Pose3d result1 = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(Math.PI);
        Pose3d result2 = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(Math.PI / 2);
        Pose3d result3 = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(1.25, result1.getZ(), EPSILON, "Z should be 0.75 + 0.50");
        assertEquals(1.25, result2.getZ(), EPSILON, "Z should be constant regardless of rotation");
        assertEquals(1.25, result3.getZ(), EPSILON, "Z should be constant regardless of rotation");
    }

    @Test
    void testGetLimelightPose_NegativeRotations() {
        // Test various negative rotation angles
        setStaticField(TurretMap.class, "LIMELIGHT_OFFSET_X", 0.3);
        setStaticField(TurretMap.class, "LIMELIGHT_PITCH", Math.toRadians(-15));
        setStaticField(TurretMap.class, "LIMELIGHT_ROLL", Math.toRadians(-5));

        setTurretRotation(Math.toRadians(-45));

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(Math.toRadians(-5), result.getRotation().getX(), EPSILON, "Roll should be -5 degrees");
        assertEquals(Math.toRadians(-15), result.getRotation().getY(), EPSILON, "Pitch should be -15 degrees");
        assertEquals(Math.toRadians(-45), result.getRotation().getZ(), EPSILON, "Yaw should be -45 degrees");
    }

    @Test
    void testGetLimelightPose_ExtremePitchAngles() {
        // Test extreme pitch angles
        setStaticField(TurretMap.class, "LIMELIGHT_PITCH", Math.toRadians(89));
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(Math.toRadians(89), result.getRotation().getY(), EPSILON, "Should handle extreme pitch");
    }

    //#endregion

    //#region Subsystem Tests

    @Test
    void testTurretSubsystem_HasGetLimelightPoseMethod() {
        // Verify the method exists and is public
        assertDoesNotThrow(() -> {
            turret.getLimelightPose3dFromRobotCenter();
        }, "getLimelightPose3dFromRobotCenter should be callable");
    }

    @Test
    void testTurretSubsystem_MethodReturnsNonNull() {
        Pose3d result = turret.getLimelightPose3dFromRobotCenter();
        assertNotNull(result, "Method should never return null");
    }

    @Test
    void testTurretSubsystem_InitializesCorrectly() {
        assertNotNull(turret, "Turret should initialize");
        assertEquals("Turret", turret.getName(), "Subsystem name should be 'Turret'");
    }

    //#endregion
}
