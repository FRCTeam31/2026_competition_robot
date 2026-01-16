package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IHopper {
    public void updateInputs(HopperInputsAutoLogged inputs);

    public void setHopper(DoubleSolenoid.Value value);
}