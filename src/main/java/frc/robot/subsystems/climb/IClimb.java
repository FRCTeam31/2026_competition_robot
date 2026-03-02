package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/** Hardware abstraction interface for the Climb subsystem. */
public interface IClimb {

    /**
     * Updates the logged inputs with the latest sensor readings.
     *
     * @param inputs The inputs object to populate
     */
    public void updateInputs(ClimbInputsAutoLogged inputs);

    /**
     * Sets the climb motor speed.
     *
     * @param speed Percent output in the range [-1, 1]
     */
    public void controlClimb(double speed);

    /**
     * Sets the support solenoid state.
     *
     * @param value The desired solenoid value
     */
    public void controlSupport(DoubleSolenoid.Value value);

    /**
     * Sets the friction brake solenoid state.
     *
     * @param value The desired solenoid value
     */
    public void controlFrictionBrake(DoubleSolenoid.Value value);

    /** Zeros the climb motor encoder position. */
    public void zeroEncoder();
}