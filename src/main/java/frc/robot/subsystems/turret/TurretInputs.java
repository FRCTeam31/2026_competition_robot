package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngularVelocity;

@AutoLog
public class TurretInputs {
    // Inputs
    public Rotation2d TurretRotation = Rotation2d.kZero;
    public boolean TurretRotationResetSwitch = true;
    public MutAngularVelocity FlywheelVelocity = RotationsPerSecond.mutable(0);
    public Angle HoodAngle = Angle.ofBaseUnits(0, Degrees);

    //States
    public Turret.FlywheelState FlywheelState = Turret.FlywheelState.STOPPED;
    public Turret.TargetingState TargetingState = Turret.TargetingState.STOPPED; // Set targeting to stopped for testing
    public Turret.LockOnState ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;
    public Turret.LockOnState YawLockedOnState = Turret.LockOnState.YAW_NOT_LOCKED_ON;
    public Turret.LockOnState FlywheelSpeedState = Turret.LockOnState.FLYWHEEL_NOT_AT_SPEED;
    public Turret.UptakeState FeedState = Turret.UptakeState.STOPPED;

    // On-target flags (computed by IO layer each update cycle)
    public boolean FlywheelAtTargetSpeed = false;
    public boolean YawOnTarget = false;
    public boolean HoodOnTarget = false;

    // Sysid
    public double FlywheelVoltage = 0;
    public double YawVoltage = 0;
}
