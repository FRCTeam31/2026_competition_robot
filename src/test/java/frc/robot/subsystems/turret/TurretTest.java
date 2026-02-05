package frc.robot.subsystems.turret;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.prime.util.MutVector;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.SuperStructure;
import frc.robot.subsystems.turret.Turret.FlywheelState;
import frc.robot.subsystems.turret.Turret.TargetingState;

/**
 * Test suite for the Turret subsystem.
 * Tests command factory methods and calculateTurretVectorFromRobotPose helper.
 */
public class TurretTest {
    private Turret turret;

    @BeforeEach
    public void setUp() {
        Assertions.assertTrue(HAL.initialize(500, 0));
        turret = new Turret(false);

        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d();
        SuperStructure.Swerve.RobotRelativeChassisSpeeds = new ChassisSpeeds();
        SuperStructure.Turret.FlywheelState = Turret.FlywheelState.STOPPED;
        SuperStructure.Turret.TargetingState = Turret.TargetingState.STOPPED;
        SuperStructure.Turret.ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;
    }

    // ============================================
    // Constructor Tests
    // ============================================

    @Test
    public void testConstructor_InitializesBothModes() {
        Assertions.assertDoesNotThrow(() -> new Turret(false));
        Assertions.assertDoesNotThrow(() -> new Turret(true));
        Assertions.assertEquals("Turret", turret.getName());
    }

    @Test
    public void testPeriodicDoesNotThrow() {
        Assertions.assertDoesNotThrow(() -> turret.periodic());
    }

    // ============================================
    // Command Factory Tests - State Transitions
    // ============================================

    @Test
    public void testFlywheelStateTransitions() {
        SuperStructure.Turret.FlywheelState = Turret.FlywheelState.STOPPED;

        turret.setFlywheel(FlywheelState.IDLE).initialize();
        Assertions.assertEquals(Turret.FlywheelState.IDLE, SuperStructure.Turret.FlywheelState);

        turret.setFlywheel(FlywheelState.SHOOTING).initialize();
        Assertions.assertEquals(Turret.FlywheelState.SHOOTING, SuperStructure.Turret.FlywheelState);

        turret.setFlywheel(FlywheelState.STOPPED).initialize();
        Assertions.assertEquals(Turret.FlywheelState.STOPPED, SuperStructure.Turret.FlywheelState);
    }

    @Test
    public void testTargetingStateTransitions() {
        SuperStructure.Turret.TargetingState = Turret.TargetingState.STOPPED;

        turret.setTargeting(TargetingState.AUTO).initialize();
        Assertions.assertEquals(Turret.TargetingState.AUTO, SuperStructure.Turret.TargetingState);

        turret.setTargeting(TargetingState.MANUAL).initialize();
        Assertions.assertEquals(Turret.TargetingState.MANUAL, SuperStructure.Turret.TargetingState);

        turret.setTargeting(TargetingState.STOPPED).initialize();
        Assertions.assertEquals(Turret.TargetingState.STOPPED, SuperStructure.Turret.TargetingState);
    }

    @Test
    public void testCommandFactoriesReturnValidCommands() {
        Assertions.assertNotNull(turret.setFlywheel(FlywheelState.SHOOTING));
        Assertions.assertNotNull(turret.setFlywheel(FlywheelState.IDLE));
        Assertions.assertNotNull(turret.setFlywheel(FlywheelState.STOPPED));
        Assertions.assertNotNull(turret.setTargeting(TargetingState.AUTO));
        Assertions.assertNotNull(turret.setTargeting(TargetingState.MANUAL));
        Assertions.assertNotNull(turret.setTargeting(TargetingState.STOPPED));
    }

    // ============================================
    // calculateTurretVectorFromRobotPose Tests
    // ============================================

    @Test
    public void testCalculateTurretVector_ReturnsSameVectorInstance() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());
        Pose3d target = new Pose3d(5, 0, 2.0, new Rotation3d());

        MutVector result1 = turret.calculateTurretVectorFromRobotPose(target);
        MutVector result2 = turret.calculateTurretVectorFromRobotPose(target);

        Assertions.assertSame(result1, result2);
    }

    @Test
    public void testCalculateTurretVector_TooCloseRejectsShot() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());
        Pose3d target = new Pose3d(0.01, 0, 2.0, new Rotation3d());

        turret.calculateTurretVectorFromRobotPose(target);

        Assertions.assertEquals(Turret.LockOnState.SHOT_NOT_CALCULATED,
                SuperStructure.Turret.ShotCalculationState);
    }

    @Test
    public void testCalculateTurretVector_ValidatesOutput() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());
        Pose3d target = new Pose3d(5, 0, 2.0, new Rotation3d());

        MutVector result = turret.calculateTurretVectorFromRobotPose(target);

        Assertions.assertFalse(Double.isNaN(result.getX()));
        Assertions.assertFalse(Double.isNaN(result.getY()));
        Assertions.assertFalse(Double.isNaN(result.getZ()));
        Assertions.assertFalse(Double.isNaN(result.getMagnitude()));
        Assertions.assertFalse(Double.isInfinite(result.getMagnitude()));
    }

    @Test
    public void testCalculateTurretVector_RepeatedCalculationsConsistent() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());
        SuperStructure.Swerve.RobotRelativeChassisSpeeds = new ChassisSpeeds();
        Pose3d target = new Pose3d(5, 0, 2.0, new Rotation3d());

        MutVector result1 = turret.calculateTurretVectorFromRobotPose(target);
        double mag1 = result1.getMagnitude();

        MutVector result2 = turret.calculateTurretVectorFromRobotPose(target);
        double mag2 = result2.getMagnitude();

        Assertions.assertEquals(mag1, mag2, 0.001);
    }

    @Test
    public void testCalculateTurretVector_HandlesAllQuadrants() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());

        // Test all four quadrants in a single test
        Pose3d[] targets = {
                new Pose3d(5, 5, 2.0, new Rotation3d()), // Quadrant I (+X, +Y)
                new Pose3d(-5, 5, 2.0, new Rotation3d()), // Quadrant II (-X, +Y)
                new Pose3d(-5, -5, 2.0, new Rotation3d()), // Quadrant III (-X, -Y)
                new Pose3d(5, -5, 2.0, new Rotation3d()) // Quadrant IV (+X, -Y)
        };

        for (Pose3d target : targets) {
            MutVector result = turret.calculateTurretVectorFromRobotPose(target);
            Assertions.assertFalse(Double.isNaN(result.getMagnitude()));
        }
    }

    @Test
    public void testCalculateTurretVector_HandlesVariousVelocities() {
        Pose3d target = new Pose3d(8, 0, 2.0, new Rotation3d());
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());

        ChassisSpeeds[] velocities = {
                new ChassisSpeeds(0, 0, 0),
                new ChassisSpeeds(5.0, 3.0, 1.5),
                new ChassisSpeeds(-1, -1, -1)
        };

        for (ChassisSpeeds velocity : velocities) {
            SuperStructure.Swerve.RobotRelativeChassisSpeeds = velocity;
            MutVector result = turret.calculateTurretVectorFromRobotPose(target);
            Assertions.assertFalse(Double.isNaN(result.getMagnitude()));
        }
    }

    @Test
    public void testCalculateTurretVector_HandlesExtremeDistances() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());

        // Near target
        turret.calculateTurretVectorFromRobotPose(new Pose3d(1.5, 0, 2.0, new Rotation3d()));
        Assertions.assertNotNull(SuperStructure.Turret.ShotCalculationState);

        // Far target - should handle gracefully
        Assertions.assertDoesNotThrow(() -> {
            turret.calculateTurretVectorFromRobotPose(new Pose3d(1000, 1000, 100.0, new Rotation3d()));
        });
    }

    @Test
    public void testCalculateTurretVector_HandlesTargetHeights() {
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(0, 0, new Rotation2d());

        // Test low, normal, and high targets
        double[] heights = { 0.0, 0.5, 2.0, 10.0 };

        for (double height : heights) {
            MutVector result = turret.calculateTurretVectorFromRobotPose(
                    new Pose3d(5, 0, height, new Rotation3d()));
            Assertions.assertFalse(Double.isNaN(result.getMagnitude()));
        }
    }
}
