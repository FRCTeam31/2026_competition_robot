package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IHopper {
    public void updateInputs(HopperInputsAutoLogged inputs);

    public void setHopper(DoubleSolenoid.Value value);

    public void toggleHopper();

    public void setFeedSpeed(Hopper.FeedState feedState);

    public void feedStop();

    public void stopIntake();

    public void setIntakePosition(Hopper.IntakeControlState controlState);

    public void setIntakeFeedState(Hopper.IntakeFeedState intakeFeedState);
}