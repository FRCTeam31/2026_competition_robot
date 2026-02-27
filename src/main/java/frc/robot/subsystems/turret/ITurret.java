package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.ControlRequest;

public interface ITurret {
    public void updateInputs(TurretInputsAutoLogged inputs);

    public void controlFlywheel(double targetRotationsPerSecond);

    public void controlYaw(ControlRequest controlRequest);

    public void controlHood(double percentOut);

    public void setFeederSpeed(double speed);

    /** Sets raw voltage to the flywheel motor (for SysId characterization) */
    public void setFlywheelVoltage(double volts);

    /** Sets raw voltage to the turret yaw motor (for SysId characterization) */
    public void setYawVoltage(double volts);
}