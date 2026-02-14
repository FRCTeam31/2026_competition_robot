package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.SuperStructure;

/**
 * Unit tests for the Turret subsystem.
 * Focuses on testing the getLimelightPose3dFromRobotCenter() function
 * to verify correct 3D pose calculation using actual TurretMap constants.
 */
class TurretTest {
    private Turret turret;
    private static final double EPSILON = 1e-6; // Tolerance for floating-point comparisons

    // Cache actual TurretMap values for readability in expected value calculations
    private double originX;
    private double originY;
    private double originZ;
    private double llOffsetX;
    private double llOffsetY;
    private double llOffsetZ;
    private double llPitch;
    private double llYaw;
    private double llRoll;

    @BeforeEach
    void setUp() {
        // Initialize HAL for WPILib
        assert HAL.initialize(500, 0);

        // Cache actual TurretMap values
        originX = TurretMap.TURRET_ROBOT_ORIGIN.getX();
        originY = TurretMap.TURRET_ROBOT_ORIGIN.getY();
        originZ = TurretMap.TURRET_ROBOT_ORIGIN.getZ();
        llOffsetX = TurretMap.LIMELIGHT_OFFSET_X;
        llOffsetY = TurretMap.LIMELIGHT_OFFSET_Y;
        llOffsetZ = TurretMap.LIMELIGHT_OFFSET_Z;
        llPitch = TurretMap.LIMELIGHT_PITCH;
        llYaw = TurretMap.LIMELIGHT_YAW;
        llRoll = TurretMap.LIMELIGHT_ROLL;

        // Create turret in simulation mode
        turret = new Turret();
    }

    //#region Helper Methods

    /**
     * Sets the turret rotation for testing
     */
    private void setTurretRotation(double radians) {
        SuperStructure.Turret.TurretRotation = new Rotation2d(radians);
    }

    /**
     * Computes the expected X position from robot center for a given turret rotation.
     * X = originX + llOffsetX * cos(theta) - llOffsetY * sin(theta)
     */
    private double expectedX(double turretRadians) {
        return originX
                + llOffsetX * Math.cos(turretRadians)
                - llOffsetY * Math.sin(turretRadians);
    }

    /**
     * Computes the expected Y position from robot center for a given turret rotation.
     * Y = originY + llOffsetX * sin(theta) + llOffsetY * cos(theta)
     */
    private double expectedY(double turretRadians) {
        return originY
                + llOffsetX * Math.sin(turretRadians)
                + llOffsetY * Math.cos(turretRadians);
    }

    /**
     * Computes the expected Z position from robot center (independent of rotation).
     * Z = originZ + llOffsetZ
     */
    private double expectedZ() {
        return originZ + llOffsetZ;
    }

    //#endregion

    //#region Zero Rotation Tests

    @Test
    void testGetLimelightPose_ZeroTurretRotation() {
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // At zero rotation, limelight offsets add directly to turret origin
        assertEquals(expectedX(0.0), result.getX(), EPSILON, "X should be origin + limelight offset X");
        assertEquals(expectedY(0.0), result.getY(), EPSILON, "Y should be origin + limelight offset Y");
        assertEquals(expectedZ(), result.getZ(), EPSILON, "Z should be origin + limelight offset Z");
        assertEquals(llRoll, result.getRotation().getX(), EPSILON, "Roll should match limelight roll");
        assertEquals(llPitch, result.getRotation().getY(), EPSILON, "Pitch should match limelight pitch");
        assertEquals(llYaw, result.getRotation().getZ(), EPSILON, "Yaw should match limelight yaw");
    }

    //#endregion

    //#region Turret Rotation Tests

    @Test
    void testGetLimelightPose_90DegreeRotation() {
        double angle = Math.PI / 2;
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(expectedX(angle), result.getX(), EPSILON, "X position at 90 degrees");
        assertEquals(expectedY(angle), result.getY(), EPSILON, "Y position at 90 degrees");
        assertEquals(expectedZ(), result.getZ(), EPSILON, "Z should be unchanged by rotation");
        assertEquals(llYaw + angle, result.getRotation().getZ(), EPSILON, "Yaw should include turret rotation");
    }

    @Test
    void testGetLimelightPose_180DegreeRotation() {
        double angle = Math.PI;
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(expectedX(angle), result.getX(), EPSILON, "X position at 180 degrees");
        assertEquals(expectedY(angle), result.getY(), EPSILON, "Y position at 180 degrees");
        assertEquals(expectedZ(), result.getZ(), EPSILON, "Z should be unchanged by rotation");
        assertEquals(llYaw + angle, result.getRotation().getZ(), EPSILON, "Yaw should be PI + limelight yaw");
    }

    @Test
    void testGetLimelightPose_NegativeRotation() {
        double angle = -Math.PI / 2;
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(expectedX(angle), result.getX(), EPSILON, "X position at -90 degrees");
        assertEquals(expectedY(angle), result.getY(), EPSILON, "Y position at -90 degrees");
        assertEquals(llYaw + angle, result.getRotation().getZ(), EPSILON, "Yaw should be -PI/2 + limelight yaw");
    }

    @Test
    void testGetLimelightPose_45DegreeRotation() {
        double angle = Math.PI / 4;
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(expectedX(angle), result.getX(), EPSILON, "X position at 45 degrees");
        assertEquals(expectedY(angle), result.getY(), EPSILON, "Y position at 45 degrees");
    }

    //#endregion

    //#region Limelight Fixed Rotation Tests

    @Test
    void testGetLimelightPose_PitchMatchesMap() {
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(llPitch, result.getRotation().getY(), EPSILON, "Pitch should match TurretMap value");
    }

    @Test
    void testGetLimelightPose_YawMatchesMapAtZeroRotation() {
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(llYaw, result.getRotation().getZ(), EPSILON,
                "Yaw at zero rotation should match TurretMap limelight yaw");
    }

    @Test
    void testGetLimelightPose_RollMatchesMap() {
        setTurretRotation(0.0);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(llRoll, result.getRotation().getX(), EPSILON, "Roll should match TurretMap value");
    }

    @Test
    void testGetLimelightPose_YawCombinesTurretRotationAndFixedYaw() {
        double turretAngle = Math.PI / 3; // 60 degrees turret rotation
        setTurretRotation(turretAngle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Combined yaw should be sum of fixed yaw and turret rotation
        assertEquals(llYaw + turretAngle, result.getRotation().getZ(), EPSILON,
                "Yaw should be limelight yaw + turret rotation");
    }

    //#endregion

    //#region Complex Scenario Tests

    @Test
    void testGetLimelightPose_ComplexRotation() {
        double angle = Math.toRadians(135);
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Verify result is not null and has reasonable values
        assertNotNull(result);
        assertTrue(Double.isFinite(result.getX()), "X should be finite");
        assertTrue(Double.isFinite(result.getY()), "Y should be finite");
        assertEquals(expectedZ(), result.getZ(), EPSILON, "Z should be origin + limelight offset Z");

        // Verify rotation components
        assertEquals(llRoll, result.getRotation().getX(), EPSILON, "Roll should match map");
        assertEquals(llPitch, result.getRotation().getY(), EPSILON, "Pitch should match map");
        assertEquals(llYaw + angle, result.getRotation().getZ(), EPSILON, "Yaw = turret rotation + limelight yaw");
    }

    @Test
    void testGetLimelightPose_RotatedOffsetPosition() {
        // With 90 degree rotation, the limelight offset X should map to Y and vice versa
        double angle = Math.PI / 2;
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Rotated offset: X' = offsetX*cos(90) - offsetY*sin(90) = -offsetY
        //                 Y' = offsetX*sin(90) + offsetY*cos(90) = offsetX
        double rotatedX = llOffsetX * Math.cos(angle) - llOffsetY * Math.sin(angle);
        double rotatedY = llOffsetX * Math.sin(angle) + llOffsetY * Math.cos(angle);

        assertEquals(originX + rotatedX, result.getX(), EPSILON, "X = origin + rotated offset X");
        assertEquals(originY + rotatedY, result.getY(), EPSILON, "Y = origin + rotated offset Y");
    }

    //#endregion

    //#region Edge Cases

    @Test
    void testGetLimelightPose_VerySmallRotation() {
        setTurretRotation(0.0);
        Pose3d baseResult = turret.getLimelightPose3dFromRobotCenter();

        double angle = 0.0001;
        setTurretRotation(angle);
        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        // Should be very close to the zero rotation case
        assertEquals(baseResult.getX(), result.getX(), 0.01, "X should be close to zero-rotation result");
        assertEquals(baseResult.getY(), result.getY(), 0.01, "Y should be close to zero-rotation result");
    }

    @Test
    void testGetLimelightPose_FullRotation() {
        // Full 360 degree rotation should give same result as zero rotation
        setTurretRotation(0.0);
        Pose3d zeroResult = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(2 * Math.PI);
        Pose3d fullResult = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(zeroResult.getX(), fullResult.getX(), EPSILON, "X should match zero rotation after full rotation");
        assertEquals(zeroResult.getY(), fullResult.getY(), EPSILON, "Y should match zero rotation after full rotation");
        assertEquals(zeroResult.getZ(), fullResult.getZ(), EPSILON, "Z should match zero rotation after full rotation");
    }

    @Test
    void testGetLimelightPose_MultipleConsecutiveCalls() {
        // Verify function is idempotent (same result on multiple calls)
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
        // Test that rotating by +angle and -angle produces expected symmetric behavior
        // cos is even, sin is odd, so the rotated X offset is the same for +/- angle
        // and the rotated Y offset is negated for +/- angle
        double angle = Math.PI / 6;

        setTurretRotation(angle);
        Pose3d resultPositive = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(-angle);
        Pose3d resultNegative = turret.getLimelightPose3dFromRobotCenter();

        // The X position should be the same for +/- angle (origin is constant, cos is even, sin is odd)
        assertEquals(resultPositive.getX(), resultNegative.getX(), EPSILON, "X should be symmetric for +/- angle");

        // The Y offset from origin should be negated for +/- angle
        double yOffsetPositive = resultPositive.getY() - originY;
        double yOffsetNegative = resultNegative.getY() - originY;
        assertEquals(yOffsetPositive, -yOffsetNegative, EPSILON,
                "Y offset from origin should be opposite for +/- angle");
    }

    @Test
    void testGetLimelightPose_ZOffsetIndependentOfRotation() {
        // Z offset should not be affected by turret rotation
        double expectedZValue = expectedZ();

        setTurretRotation(0.0);
        Pose3d result1 = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(Math.PI);
        Pose3d result2 = turret.getLimelightPose3dFromRobotCenter();

        setTurretRotation(Math.PI / 2);
        Pose3d result3 = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(expectedZValue, result1.getZ(), EPSILON, "Z should be constant at 0 rotation");
        assertEquals(expectedZValue, result2.getZ(), EPSILON, "Z should be constant at PI rotation");
        assertEquals(expectedZValue, result3.getZ(), EPSILON, "Z should be constant at PI/2 rotation");
    }

    @Test
    void testGetLimelightPose_NegativeRotations() {
        double angle = Math.toRadians(-45);
        setTurretRotation(angle);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertEquals(llRoll, result.getRotation().getX(), EPSILON, "Roll should match map value");
        assertEquals(llPitch, result.getRotation().getY(), EPSILON, "Pitch should match map value");
        assertEquals(llYaw + angle, result.getRotation().getZ(), EPSILON,
                "Yaw should be limelight yaw + turret rotation");
    }

    @Test
    void testGetLimelightPose_ResultIsFiniteForLargeRotation() {
        setTurretRotation(Math.PI / 4);

        Pose3d result = turret.getLimelightPose3dFromRobotCenter();

        assertNotNull(result);
        assertTrue(Double.isFinite(result.getX()), "X should be finite");
        assertTrue(Double.isFinite(result.getY()), "Y should be finite");
        assertTrue(Double.isFinite(result.getZ()), "Z should be finite");
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
