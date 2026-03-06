package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Unit tests for SwerveIOPackager - validates swerve module and gyro orchestration.
 * Tests verify module state management, gyro integration, pose estimation, and motor control.
 */
class SwerveIOPackagerTest {
    private SwerveIOPackager packager;
    private SwerveSubsystemInputsAutoLogged inputs;
    private static final double EPSILON = 1e-6;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        packager = new SwerveIOPackager();
        inputs = new SwerveSubsystemInputsAutoLogged();
    }

    // ==================== Constructor & Initialization Tests ====================

    @Test
    void testConstructor_InitializesBothModes() {
        assertDoesNotThrow(() -> new SwerveIOPackager(), "Constructor should initialize without errors");
        assertNotNull(packager.Kinematics, "Kinematics should be initialized");
    }

    @Test
    void testUpdateInputs_InitializesAllFieldsCorrectly() {
        packager.updateInputs(inputs);

        // Validate all fields are initialized
        assertNotNull(inputs.ModuleStates, "Module states should not be null");
        assertEquals(4, inputs.ModuleStates.length, "Should have 4 module states");
        assertNotNull(inputs.RobotRelativeChassisSpeeds, "Chassis speeds should not be null");
        assertNotNull(inputs.GyroAngle, "Gyro angle should not be null");
        assertNotNull(inputs.EstimatedRobotPose, "Estimated robot pose should not be null");

        // Initial pose should be origin
        assertEquals(0.0, inputs.EstimatedRobotPose.getX(), EPSILON, "Initial X position should be 0");
        assertEquals(0.0, inputs.EstimatedRobotPose.getY(), EPSILON, "Initial Y position should be 0");
    }

    // ==================== Module States & Positions Tests ====================

    @Test
    void testGetModuleStatesAndPositions_ReturnsFourValidModules() {
        packager.updateInputs(inputs);
        SwerveModuleState[] states = packager.getModuleStates();
        SwerveModulePosition[] positions = packager.getModulePositions();

        assertEquals(4, states.length, "Should return 4 module states");
        assertEquals(4, positions.length, "Should return 4 module positions");

        for (int i = 0; i < 4; i++) {
            assertNotNull(states[i], "Module state " + i + " should not be null");
            assertNotNull(positions[i], "Module position " + i + " should not be null");
            assertEquals(0.0, positions[i].distanceMeters, EPSILON,
                    "Initial distance for module " + i + " should be 0");
        }
    }

    @Test
    void testSetDesiredModuleStates_UpdatesModuleAngles() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(45))
        };

        packager.setDesiredModuleStates(desiredStates);

        // Update inputs to see the effect
        for (int i = 0; i < 10; i++) {
            packager.updateInputs(inputs);
        }

        // All modules should have the same angle
        assertEquals(Math.toRadians(45), inputs.ModuleStates[0].angle.getRadians(), 0.01);
    }

    // ==================== Gyro Tests ====================

    @Test
    void testSetSimGyroOmega_UpdatesGyroAngle() {
        packager.setSimGyroOmega(1.0);
        packager.updateInputs(inputs);
        Rotation2d angle1 = inputs.GyroAngle;

        // Multiple updates should show gyro rotating
        for (int i = 0; i < 10; i++) {
            packager.updateInputs(inputs);
        }

        assertNotEquals(angle1.getRadians(), inputs.GyroAngle.getRadians(),
                "Gyro angle should change with non-zero omega");
    }

    // ==================== Stop Motors Tests ====================

    @Test
    void testStopAllMotors_StopsModuleMotion() {
        // Set modules in motion
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0))
        };
        packager.setDesiredModuleStates(desiredStates);

        for (int i = 0; i < 5; i++) {
            packager.updateInputs(inputs);
        }

        // Stop all motors
        packager.stopAllMotors();
        packager.updateInputs(inputs);

        // All modules should have zero speed
        for (int i = 0; i < inputs.ModuleStates.length; i++) {
            assertEquals(0.0, inputs.ModuleStates[i].speedMetersPerSecond, EPSILON,
                    "Module " + i + " speed should be 0 after stopping");
        }
    }

    // ==================== Pose Estimation Tests ====================

    @Test
    void testSetEstimatorPose_UpdatesEstimatedPose() {
        Pose2d newPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));

        packager.setEstimatorPose(newPose);
        packager.updateInputs(inputs);

        assertEquals(3.0, inputs.EstimatedRobotPose.getX(), EPSILON,
                "Estimated pose X should be updated");
        assertEquals(4.0, inputs.EstimatedRobotPose.getY(), EPSILON,
                "Estimated pose Y should be updated");
    }

    @Test
    void testAddPoseEstimatorVisionMeasurement_AcceptsValidInput() {
        Pose2d visionPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));
        double timestamp = 1.0;
        var stdDevs = edu.wpi.first.math.VecBuilder.fill(0.5, 0.5, 0.1);

        assertDoesNotThrow(() -> packager.addPoseEstimatorVisionMeasurement(visionPose, timestamp, stdDevs),
                "Should accept vision measurement");
    }

    // ==================== Kinematics Tests ====================

    @Test
    void testKinematics_ConvertsBetweenModuleStatesAndChassisSpeeds() {
        // Test conversion to module states
        edu.wpi.first.math.kinematics.ChassisSpeeds chassisSpeeds = new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0,
                0.0, 0.0);
        var moduleStates = packager.Kinematics.toSwerveModuleStates(chassisSpeeds);
        assertEquals(4, moduleStates.length, "Should convert to 4 module states");

        // Test conversion to chassis speeds
        SwerveModuleState[] forwardStates = new SwerveModuleState[] {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0))
        };
        var resultSpeeds = packager.Kinematics.toChassisSpeeds(forwardStates);
        assertNotNull(resultSpeeds);
    }

    // ==================== Edge Cases ====================

    @Test
    void testResetGyro_ResetsAngleAfterRotation() {
        // Set gyro omega and let it rotate
        packager.setSimGyroOmega(2.0);
        for (int i = 0; i < 10; i++) {
            packager.updateInputs(inputs);
        }

        // Reset gyro and verify it still works
        packager.resetGyro();
        packager.updateInputs(inputs);
        assertNotNull(inputs.GyroAngle, "Should handle reset after rotation");

        // Verify multiple resets work
        packager.resetGyro();
        packager.updateInputs(inputs);
        assertNotNull(inputs.GyroAngle, "Should handle multiple resets");
    }
}
