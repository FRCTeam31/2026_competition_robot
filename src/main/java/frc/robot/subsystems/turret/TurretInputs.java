package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.subsystems.turret.Turret.FlywheelStates;
import frc.robot.subsystems.turret.Turret.TargetingStates;

@AutoLog
public class TurretInputs {
    // Inputs
    public Rotation2d TurretRotation;
    public boolean TurretRotationResetSwitch;
    public MutAngularVelocity FlywheelVelocity;

    //States
    public FlywheelStates FlywheelStates;
    public TargetingStates TargetingState;
}
