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
    public double TurretRotationDegrees = 0;
    public boolean TurretRotationResetSwitch = true;
    public MutAngularVelocity FlywheelVelocity = RotationsPerSecond.mutable(0);
    public double FlywheelVelocityRPM = 0;
    public Angle HoodAngle = Degrees.of(0);
    public Angle FlywheelAngle = Degrees.of(0);

    // Operating mode (persists across firing cycles)
    // TODO: Changed to prevent movement while testing
    public Turret.OperatingMode OperatingMode = Turret.OperatingMode.MANUAL;

    // Firing state (ephemeral: FIRING while button held, IDLE when released)
    public Turret.FiringState FiringState = Turret.FiringState.IDLE;

    // Lock-on tracking
    public Turret.LockOnState ShotCalculationState = Turret.LockOnState.SHOT_NOT_CALCULATED;

    // Feed direction
    public Turret.UptakeState FeedState = Turret.UptakeState.STOPPED;

    // On-target flags (computed by IO layer each update cycle)
    public boolean FlywheelAtTargetSpeed = false;
    public boolean YawOnTarget = false;
    public boolean HoodOnTarget = false;

    // Sysid
    public double FlywheelVoltage = 0;
    public double YawVoltage = 0;
}
