package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.MutAngularVelocity;

@AutoLog
public class TurretInputs {
    // Inputs
    public Rotation2d TurretRotation = Rotation2d.kZero;
    public boolean TurretRotationResetSwitch = true;
    public MutAngularVelocity FlywheelVelocity = RotationsPerSecond.mutable(0);
    public Pose3d LimelightPoseFromRobotCenter = Pose3d.kZero;

    //States
    public Turret.FlywheelState FlywheelState = Turret.FlywheelState.STOPPED;
    public Turret.TargetingState TargetingState = Turret.TargetingState.AUTO;
    public Turret.LockOnState ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;
    public Turret.LockOnState YawLockedOnState = Turret.LockOnState.YAW_NOT_LOCKED_ON;
    public Turret.LockOnState FlywheelSpeedState = Turret.LockOnState.FLYWHEEL_NOT_AT_SPEED;
    public Turret.UptakeState FeedState = Turret.UptakeState.STOPPED;

    // Targets
    public Translation3d targetVector = Translation3d.kZero;

    // Sysid
    public double FlywheelVoltage = 0;
    public double YawVoltage = 0;
}
