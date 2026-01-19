package frc.robot.subsystems.climb;

public interface IClimb {
    public void updateInputs(ClimbInputsAutoLogged inputs);

    public void controlClimb(Climb.ClimbState state);

    public void controlSupport(Climb.SupportState state);
}