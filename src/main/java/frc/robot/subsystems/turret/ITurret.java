package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ITurret {
    public void updateInputs(TurretInputsAutoLogged inputs);

    public void controlFlywheel(AngularVelocity velocity);

    public void controlYawAngle(Angle angle);

    public void setYawPercentOut(double percentOut);

    public void controlHood(Angle angle);

    public void setHoodPercentOut(double percentOut);

    public void setFeederSpeed(double speed);

    /** Sets raw voltage to the flywheel motor (for SysId characterization) */
    public void setFlywheelVoltage(double volts);

    /** Sets raw voltage to the turret yaw motor (for SysId characterization) */
    public void setYawVoltage(double volts);
}