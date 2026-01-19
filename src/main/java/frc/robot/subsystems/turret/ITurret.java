package frc.robot.subsystems.turret;

import frc.robot.subsystems.turret.Turret.FlywheelStates;
import frc.robot.subsystems.turret.Turret.TargetingStates;

public interface ITurret {
    public void updateInputs(TurretInputsAutoLogged inputs);

    public void controlFlywheel(FlywheelStates state, double manualTargetVelocityRPS);

    public void controlTargeting(TargetingStates state, double manualControlSpeed);

    public void setFeederSpeed(double speed);

}