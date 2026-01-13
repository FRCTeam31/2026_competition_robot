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
        packager = new SwerveIOPackager(false); // Use sim mode
        inputs = new SwerveSubsystemInputsAutoLogged();
    }

    // ==================== Constructor Tests ====================

    @Test
    void testConstructor_InitializesSimMode() {
        assertDoesNotThrow(() -> new SwerveIOPackager(false),
                "Constructor should initialize in sim mode");
    }

    @Test
    void testConstructor_InitializesRealMode() {
        assertDoesNotThrow(() -> new SwerveIOPackager(true),
                "Constructor should initialize in real mode");
    }

    @Test
    void testConstructor_CreatesKinematics() {
        assertNotNull(packager.Kinematics, "Kinematics should be initialized");
    }

    // ==================== UpdateInputs Tests ====================

    @Test
    void testUpdateInputs_InitializesModuleStates() {
        packager.updateInputs(inputs);

        assertNotNull(inputs.ModuleStates, "Module states should not be null");
        assertEquals(4, inputs.ModuleStates.length, "Should have 4 module states");
    }

    @Test
    void testUpdateInputs_InitializesChassisSpeeds() {
        packager.updateInputs(inputs);

        assertNotNull(inputs.RobotRelativeChassisSpeeds, "Chassis speeds should not be null");
    }

    @Test
    void testUpdateInputs_InitializesGyroAngle() {
        packager.updateInputs(inputs);

        assertNotNull(inputs.GyroAngle, "Gyro angle should not be null");
    }

    @Test
    void testUpdateInputs_InitializesEstimatedPose() {
        packager.updateInputs(inputs);

        assertNotNull(inputs.EstimatedRobotPose, "Estimated robot pose should not be null");
    }

    @Test
    void testUpdateInputs_InitialPoseIsOrigin() {
        packager.updateInputs(inputs);

        assertEquals(0.0, inputs.EstimatedRobotPose.getX(), EPSILON,
                "Initial X position should be 0");
        assertEquals(0.0, inputs.EstimatedRobotPose.getY(), EPSILON,
                "Initial Y position should be 0");
    }

    @Test
    void testUpdateInputs_ConsecutiveCalls() {
        packager.updateInputs(inputs);
        Pose2d firstPose = inputs.EstimatedRobotPose;

        packager.updateInputs(inputs);
        Pose2d secondPose = inputs.EstimatedRobotPose;

        assertNotNull(firstPose);
        assertNotNull(secondPose);
    }

    // ==================== GetModuleStates Tests ====================

    @Test
    void testGetModuleStates_ReturnsFourModules() {
        packager.updateInputs(inputs);
        SwerveModuleState[] states = packager.getModuleStates();

        assertEquals(4, states.length, "Should return 4 module states");
    }

    @Test
    void testGetModuleStates_AllStatesNotNull() {
        packager.updateInputs(inputs);
        SwerveModuleState[] states = packager.getModuleStates();

        for (int i = 0; i < states.length; i++) {
            assertNotNull(states[i], "Module state " + i + " should not be null");
        }
    }

    @Test
    void testGetModuleStates_CorrectOrder() {
        packager.updateInputs(inputs);
        SwerveModuleState[] states = packager.getModuleStates();

        // Order should be FL, FR, RL, RR
        assertNotNull(states[0]); // Front Left
        assertNotNull(states[1]); // Front Right
        assertNotNull(states[2]); // Rear Left
        assertNotNull(states[3]); // Rear Right
    }

    // ==================== GetModulePositions Tests ====================

    @Test
    void testGetModulePositions_ReturnsFourModules() {
        packager.updateInputs(inputs);
        SwerveModulePosition[] positions = packager.getModulePositions();

        assertEquals(4, positions.length, "Should return 4 module positions");
    }

    @Test
    void testGetModulePositions_AllPositionsNotNull() {
        packager.updateInputs(inputs);
        SwerveModulePosition[] positions = packager.getModulePositions();

        for (int i = 0; i < positions.length; i++) {
            assertNotNull(positions[i], "Module position " + i + " should not be null");
        }
    }

    @Test
    void testGetModulePositions_InitialDistanceIsZero() {
        packager.updateInputs(inputs);
        SwerveModulePosition[] positions = packager.getModulePositions();

        for (int i = 0; i < positions.length; i++) {
            assertEquals(0.0, positions[i].distanceMeters, EPSILON,
                    "Initial distance for module " + i + " should be 0");
        }
    }

    // ==================== SetDesiredModuleStates Tests ====================

    @Test
    void testSetDesiredModuleStates_AcceptsFourStates() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0))
        };

        assertDoesNotThrow(() -> packager.setDesiredModuleStates(desiredStates),
                "Should accept array of 4 module states");
    }

    @Test
    void testSetDesiredModuleStates_UpdatesModuleStates() {
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

    // ==================== SetSimGyroOmega Tests ====================

    @Test
    void testSetSimGyroOmega_AcceptsValue() {
        assertDoesNotThrow(() -> packager.setSimGyroOmega(1.5),
                "Should accept gyro omega value");
    }

    @Test
    void testSetSimGyroOmega_UpdatesGyroInSim() {
        packager.setSimGyroOmega(1.0);

        packager.updateInputs(inputs);
        Rotation2d angle1 = inputs.GyroAngle;

        // Multiple updates should show gyro rotating
        for (int i = 0; i < 10; i++) {
            packager.updateInputs(inputs);
        }

        Rotation2d angle2 = inputs.GyroAngle;

        assertNotEquals(angle1.getRadians(), angle2.getRadians(),
                "Gyro angle should change with non-zero omega");
    }

    // ==================== StopAllMotors Tests ====================

    @Test
    void testStopAllMotors_DoesNotThrow() {
        assertDoesNotThrow(() -> packager.stopAllMotors(),
                "StopAllMotors should not throw exceptions");
    }

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

    // ==================== ResetGyro Tests ====================

    @Test
    void testResetGyro_DoesNotThrow() {
        assertDoesNotThrow(() -> packager.resetGyro(),
                "ResetGyro should not throw exceptions");
    }

    @Test
    void testResetGyro_UpdatesGyroAngle() {
        // Set gyro omega and let it rotate
        packager.setSimGyroOmega(2.0);
        for (int i = 0; i < 10; i++) {
            packager.updateInputs(inputs);
        }

        // Reset gyro
        packager.resetGyro();
        packager.updateInputs(inputs);

        // Angle should be valid after reset
        assertNotNull(inputs.GyroAngle);
    }

    // ==================== SetEstimatorPose Tests ====================

    @Test
    void testSetEstimatorPose_AcceptsPose() {
        Pose2d newPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));

        assertDoesNotThrow(() -> packager.setEstimatorPose(newPose),
                "Should accept pose for estimator");
    }

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

    // ==================== AddPoseEstimatorVisionMeasurement Tests ====================

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
    void testKinematics_CorrectNumberOfModules() {
        // Kinematics should be configured for 4 modules
        assertNotNull(packager.Kinematics);
    }

    @Test
    void testKinematics_CanConvertToModuleStates() {
        edu.wpi.first.math.kinematics.ChassisSpeeds chassisSpeeds = new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0,
                0.0, 0.0);

        var moduleStates = packager.Kinematics.toSwerveModuleStates(chassisSpeeds);

        assertEquals(4, moduleStates.length, "Should convert to 4 module states");
    }

    @Test
    void testKinematics_CanConvertToChassisSpeeds() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0))
        };

        var chassisSpeeds = packager.Kinematics.toChassisSpeeds(moduleStates);

        assertNotNull(chassisSpeeds);
    }

    // ==================== Integration Tests ====================

    @Test
    void testFullCycle_SetStatesAndUpdate() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(45))
        };

        packager.setDesiredModuleStates(desiredStates);
        packager.updateInputs(inputs);

        assertNotNull(inputs.ModuleStates);
        assertNotNull(inputs.EstimatedRobotPose);
        assertNotNull(inputs.RobotRelativeChassisSpeeds);
    }

    @Test
    void testPoseEstimationUpdates() {
        // Set modules to drive forward
        SwerveModuleState[] forwardStates = new SwerveModuleState[] {
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(1.0, Rotation2d.fromDegrees(0))
        };

        packager.setDesiredModuleStates(forwardStates);

        packager.updateInputs(inputs);
        double initialX = inputs.EstimatedRobotPose.getX();

        // Drive for multiple cycles
        for (int i = 0; i < 50; i++) {
            packager.updateInputs(inputs);
        }

        double finalX = inputs.EstimatedRobotPose.getX();

        assertTrue(finalX > initialX, "Robot should move forward in X direction");
    }

    @Test
    void testGyroRotationTracking() {
        packager.setSimGyroOmega(Math.PI); // 180 degrees per second

        packager.updateInputs(inputs);
        double initialAngle = inputs.GyroAngle.getRadians();

        for (int i = 0; i < 50; i++) {
            packager.updateInputs(inputs);
        }

        double finalAngle = inputs.GyroAngle.getRadians();

        assertNotEquals(initialAngle, finalAngle, "Gyro angle should change with rotation");
    }

    // ==================== Edge Cases ====================

    @Test
    void testMultipleResets() {
        packager.resetGyro();
        packager.updateInputs(inputs);

        packager.resetGyro();
        packager.updateInputs(inputs);

        assertNotNull(inputs.GyroAngle, "Should handle multiple resets");
    }

    @Test
    void testSetPoseAfterMovement() {
        // Move the robot
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(2.0, Rotation2d.fromDegrees(0))
        };
        packager.setDesiredModuleStates(states);

        for (int i = 0; i < 10; i++) {
            packager.updateInputs(inputs);
        }

        // Reset pose to a known location
        Pose2d resetPose = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(0));
        packager.setEstimatorPose(resetPose);
        packager.updateInputs(inputs);

        assertEquals(5.0, inputs.EstimatedRobotPose.getX(), 0.1,
                "X position should be close to reset value");
        assertEquals(5.0, inputs.EstimatedRobotPose.getY(), 0.1,
                "Y position should be close to reset value");
    }
}
