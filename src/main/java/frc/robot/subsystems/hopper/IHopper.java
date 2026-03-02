package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IHopper {
    public void updateInputs(HopperInputsAutoLogged inputs);

    public void setFeedSpeed(double speed);

    public void feedStop();

    public void stopIntake();

    public void setIntakePosition(DoubleSolenoid.Value value);

    public void setIntakeFeedSpeed(double speed);
}