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
     * Commands the climb motor to a position setpoint using MAXMotion.
     *
     * @param rotations The target position in motor rotations
     */
    public void setClimbPosition(double rotations);

    /**
     * Stops the climb motor and holds the current position.
     */
    public void stopClimb();

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