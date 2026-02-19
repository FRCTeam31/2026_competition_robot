package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.SuperStructure;

/**
 * Subsystem-level tests for Turret methods that interact with SuperStructure state.
 * Pure math tests live in {@link org.prime.subsystems.turret.TurretUtilitiesTest}.
 */
class TurretMathTest {

    private Turret _turret;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        // Reset SuperStructure state for clean tests
        SuperStructure.Swerve.EstimatedRobotPose = new Pose2d();
        SuperStructure.Turret.TurretRotation = Rotation2d.kZero;
        SuperStructure.Turret.ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;

        _turret = new Turret();
    }

    // =========================================================================
    // calculateTurretVectorFromRobotPose tests
    // =========================================================================

    @Nested
    class CalculateTurretVectorFromRobotPoseTests {

        /**
         * After a successful calculation, SuperStructure.Turret.ShotCalculationState
         * should be SHOT_CALCULATED.
         */
        @Test
        void setsState_ToShotCalculated_OnSuccess() {
            SuperStructure.Swerve.EstimatedRobotPose = new Pose2d();
            SuperStructure.Turret.ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;

            Pose3d turretPose = new Pose3d(0, 0, TurretMap.TURRET_HEIGHT_ABOVE_GROUND, Rotation3d.kZero);
            Pose3d targetPose = new Pose3d(5, 0, 2.5, Rotation3d.kZero);

            _turret.calculateTurretVectorFromRobotPose(targetPose, turretPose);

            assertEquals(Turret.LockOnState.SHOT_CALCULATED,
                    SuperStructure.Turret.ShotCalculationState,
                    "Shot state should be SHOT_CALCULATED after a valid calculation");
        }

        /**
         * When the target is impossibly far/requires too much speed with default limits,
         * the state should remain SHOT_NOT_CALCULATED.
         */
        @Test
        void setsState_ToShotNotCalculated_OnFailure() {
            SuperStructure.Swerve.EstimatedRobotPose = new Pose2d();
            SuperStructure.Turret.ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;

            Pose3d turretPose = new Pose3d(0, 0, TurretMap.TURRET_HEIGHT_ABOVE_GROUND, Rotation3d.kZero);
            // Target extremely far away — will exceed FLYWHEEL_MAX_SPEED
            Pose3d targetPose = new Pose3d(200, 0, 100, Rotation3d.kZero);

            _turret.calculateTurretVectorFromRobotPose(targetPose, turretPose);

            assertEquals(Turret.LockOnState.SHOT_NOT_CALCULATED,
                    SuperStructure.Turret.ShotCalculationState,
                    "Shot state should remain SHOT_NOT_CALCULATED when aim is impossible");
        }

        /**
         * The method should not throw even when the internal calculation fails;
         * it should catch exceptions and set state accordingly.
         */
        @Test
        void doesNotThrow_OnImpossibleGeometry() {
            SuperStructure.Swerve.EstimatedRobotPose = new Pose2d();

            Pose3d turretPose = new Pose3d(0, 0, TurretMap.TURRET_HEIGHT_ABOVE_GROUND, Rotation3d.kZero);
            Pose3d targetPose = new Pose3d(0, 0, 100, Rotation3d.kZero); // directly above = degenerate

            assertDoesNotThrow(
                    () -> _turret.calculateTurretVectorFromRobotPose(targetPose, turretPose),
                    "Should not throw; exceptions are caught internally");
        }

        /**
         * When the robot has a non-origin pose, the distance calculation
         * should still produce a valid shot for a reachable target.
         */
        @Test
        void worksWithNonOriginRobotPose() {
            SuperStructure.Swerve.EstimatedRobotPose = new Pose2d(2, 3, Rotation2d.fromDegrees(45));

            Pose3d turretPose = new Pose3d(2, 3, TurretMap.TURRET_HEIGHT_ABOVE_GROUND, Rotation3d.kZero);
            Pose3d targetPose = new Pose3d(7, 3, 2.5, Rotation3d.kZero);

            _turret.calculateTurretVectorFromRobotPose(targetPose, turretPose);

            assertEquals(Turret.LockOnState.SHOT_CALCULATED,
                    SuperStructure.Turret.ShotCalculationState,
                    "Should calculate shot from non-origin robot pose");
        }
    }

}
