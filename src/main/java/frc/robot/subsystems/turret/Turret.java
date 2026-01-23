package frc.robot.subsystems.turret;

import org.prime.util.MutVector;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Container;
import frc.robot.SuperStructure;

public class Turret extends SubsystemBase {
    private ITurret _turret;

    public enum FlywheelStates {
        IDLE,
        SHOOT_MANUAL,
        SHOOT_AUTO,
        PASSING,
        STOPPED
    }

    public enum TargetingStates {
        MANUAL_CONTROL,
        AUTO_ASSISTED
    }

    public Turret(boolean isReal) {
        _turret = isReal ? new TurretReal() : new TurretSim();
    }

    @Override
    public void periodic() {
        _turret.updateInputs(SuperStructure.Turret);
    }

    // Mutable Vectors
    private final MutVector _mutNominalTargetVector = new MutVector();
    private final MutVector _mutRobotVelocityVector = new MutVector();
    private final MutVector _mutTurretTangentVelocityVector = new MutVector();

    public MutVector calculateTargetVector() {
        var robotPose = SuperStructure.Swerve.EstimatedRobotPose;
        var deltaX = robotPose.getX() - TurretMap.HUB_GOAL_POSITION.getX();
        var deltaY = robotPose.getY() - TurretMap.HUB_GOAL_POSITION.getY();

        var yaw = Math.atan(deltaY / deltaX);

        var distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        var hubHeight = TurretMap.HUB_GOAL_POSITION.getY();
        var maxHeight = hubHeight + TurretMap.HUB_OVERSHOOT_HEIGHT;
        var turretHeight = TurretMap.TURRET_HEIGHT_ABOVE_GROUND;

        double pitch = Math.atan(
                (2 * (maxHeight - turretHeight) + Math.sqrt((maxHeight - turretHeight) * (maxHeight - turretHeight)))
                        / distance);
        double velocity = Math.sqrt(2 * 9.81 * (hubHeight - turretHeight)) / Math.sin(pitch);

        _mutNominalTargetVector.setPolar(velocity, pitch, yaw);
        return _mutNominalTargetVector;
    }

    public MutVector calculateTurretAimVector() {
        calculateTargetVector();

        if (TurretMap.AUTO_MOTION_COMPENSATION) {
            ChassisSpeeds chassisSpeeds = SuperStructure.Swerve.RobotRelativeChassisSpeeds;

            _mutRobotVelocityVector.setCartesian(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    0);

            _mutTurretTangentVelocityVector.setPolar(
                    chassisSpeeds.omegaRadiansPerSecond * TurretMap.TURRET_DISTANCE_FROM_ROBOT_CENTER,
                    0,
                    TurretMap.TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT.getDegrees());

            return _mutNominalTargetVector
                    .minus(_mutRobotVelocityVector.plus(_mutTurretTangentVelocityVector));
        } else {
            return _mutNominalTargetVector;
        }
    }
}
