package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;

/**
 * Simulation implementation of the Climb subsystem IO.
 * Models MAXMotion position control by moving toward a target position each timestep.
 */
public class ClimbSim implements IClimb {

    // Simulated device state
    private double _targetRotations = 0;
    private double _motorPosition = 0;
    private boolean _isStopped = true;
    private boolean _lowerLimitSwitch = false;
    private DoubleSolenoid.Value _frictionBrakeState = DoubleSolenoid.Value.kOff;
    @SuppressWarnings("unused")
    private DoubleSolenoid.Value _supportState = DoubleSolenoid.Value.kOff;

    public ClimbSim() {
    }

    /**
     * Advances the simulation by one timestep. Moves toward the target position
     * at the configured cruise velocity, unless the friction brake is engaged.
     */
    private void updateSimulation(double timestepSeconds) {
        // Don't move if stopped or friction brake is engaged
        if (_isStopped || _frictionBrakeState == DoubleSolenoid.Value.kForward) {
            return;
        }

        // Simulate movement toward target at cruise velocity
        double error = _targetRotations - _motorPosition;
        double maxStep = ClimbMap.MAX_MOTION_MAX_VELOCITY * timestepSeconds;

        if (Math.abs(error) <= maxStep) {
            _motorPosition = _targetRotations;
        } else {
            _motorPosition += Math.signum(error) * maxStep;
        }

        // Clamp to soft limits
        _motorPosition = Math.max(ClimbMap.REVERSE_SOFT_LIMIT, Math.min(ClimbMap.FORWARD_SOFT_LIMIT, _motorPosition));

        // Trigger lower limit switch near home
        _lowerLimitSwitch = _motorPosition <= ClimbMap.RETRACTED_ROTATIONS;
    }

    @Override
    public void updateInputs(ClimbInputsAutoLogged inputs) {
        updateSimulation(Robot.defaultPeriodSecs);

        inputs.LowerLimitSwitch = _lowerLimitSwitch;
        inputs.MotorRotations = _motorPosition;
    }

    @Override
    public void setClimbPosition(double rotations) {
        _targetRotations = rotations;
        _isStopped = false;
    }

    @Override
    public void stopClimb() {
        _isStopped = true;
    }

    @Override
    public void controlSupport(DoubleSolenoid.Value value) {
        _supportState = value;
    }

    @Override
    public void controlFrictionBrake(DoubleSolenoid.Value value) {
        _frictionBrakeState = value;
    }

    @Override
    public void zeroEncoder() {
        _motorPosition = 0;
    }

    @Override
    public void setClimbPercentOut(double percentOut) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setClimbPercentOut'");
    }
}