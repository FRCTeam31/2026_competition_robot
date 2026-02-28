package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IClimb {
    public void updateInputs(ClimbInputsAutoLogged inputs);

    public void controlClimb(double speed);

    public void controlSupport(DoubleSolenoid.Value value);

    public void controlFrictionBrake(DoubleSolenoid.Value value);

    public void zeroEncoder();
}