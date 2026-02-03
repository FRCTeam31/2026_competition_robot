package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngularVelocity;

@AutoLog
public class TurretInputs {
    // Inputs
    public Rotation2d TurretRotation = Rotation2d.kZero;
    public boolean TurretRotationResetSwitch = true;
    public MutAngularVelocity FlywheelVelocity = RotationsPerSecond.mutable(0);

    //States
    public Turret.FlywheelState FlywheelState = Turret.FlywheelState.STOPPED;
    public Turret.TargetingState TargetingState = Turret.TargetingState.AUTO;
}
