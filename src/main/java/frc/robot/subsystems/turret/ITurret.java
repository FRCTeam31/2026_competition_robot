package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.ControlRequest;

public interface ITurret {
    public void updateInputs(TurretInputsAutoLogged inputs);

    public void controlFlywheel(ControlRequest controlRequest);

    public void controlYaw(ControlRequest controlRequest);

    public void controlHood(double percentOut);

    public void setFeederSpeed(double speed);

}