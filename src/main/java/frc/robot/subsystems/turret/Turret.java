package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public Translation3d getTargetVector() {

        var robotPose = SuperStructure.Swerve.EstimatedRobotPose;
        var deltaX = robotPose.getX() - TurretMap.HUB_GOAL_POSITION.getX();
        var deltaY = robotPose.getY() - TurretMap.HUB_GOAL_POSITION.getY();
        var deltaZ = TurretMap.HUB_GOAL_POSITION.getZ();

        var yaw = Math.atan(deltaY / deltaX);
        var pitch = Math.atan(deltaZ / Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));

        // TODO: Finish implementation

        return null;

    }

    public Translation3d turretLogic() {
        Translation3d targetVector = getTargetVector();

        // TODO: Ideally find a way to avoid creating new Vector's every call

        var chassisSpeeds = SuperStructure.Swerve.RobotRelativeChassisSpeeds;
        Translation3d robotVelocityVector = new Translation3d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond, 0);
        Translation3d robotRotationVector = new Translation3d(
                chassisSpeeds.omegaRadiansPerSecond * TurretMap.TURRET_DISTANCE_FROM_ROBOT_CENTER,
                new Rotation3d(TurretMap.TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT));

        Translation3d finalTargetVector = targetVector.minus(robotVelocityVector.plus(robotRotationVector));

        return finalTargetVector;
    }
}
