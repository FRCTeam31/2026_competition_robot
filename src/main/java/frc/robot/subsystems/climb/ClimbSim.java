package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;

public class ClimbSim implements IClimb {

    private double _motorSpeed = 0;
    private double _motorPosition = 0;

    private boolean _upperLimitSwitch = false;
    private boolean _lowerLimitSwitch = false;

    private DoubleSolenoid.Value _frictionBrakeState = DoubleSolenoid.Value.kOff;
    @SuppressWarnings("unused")
    private DoubleSolenoid.Value _supportState = DoubleSolenoid.Value.kOff;

    // Simulate position bounds for limit switches
    private static final double MAX_POSITION = 1.0; // Adjust to match physical range
    private static final double MIN_POSITION = 0.0;

    public ClimbSim() {
    }

    /**
     * Call this in your sim loop (e.g. from the subsystem's simulationPeriodic).
     * @param timestepSeconds the simulation timestep (typically 0.02s)
     */
    public void updateSimulation(double timestepSeconds) {
        // Only allow motor to run if friction brake is not engaged
        if (_frictionBrakeState != DoubleSolenoid.Value.kForward) {
            _motorPosition += _motorSpeed * timestepSeconds;
        }

        // Clamp position and trigger limit switches
        if (_motorPosition >= MAX_POSITION) {
            _motorPosition = MAX_POSITION;
            _upperLimitSwitch = true;
        } else {
            _upperLimitSwitch = false;
        }

        if (_motorPosition <= MIN_POSITION) {
            _motorPosition = MIN_POSITION;
            _lowerLimitSwitch = true;
        } else {
            _lowerLimitSwitch = false;
        }
    }

    @Override
    public void updateInputs(ClimbInputsAutoLogged inputs) {
        updateSimulation(Robot.defaultPeriodSecs);

        inputs.upperLimitSwitch = _upperLimitSwitch;
        inputs.lowerLimitSwitch = _lowerLimitSwitch;

        inputs.climbExtension = inputs.climbState == Climb.ClimbState.UP
                ? 0.2794
                : 0;
        inputs.supportAngle = inputs.supportState == Climb.SupportState.LOWERED
                ? Rotation2d.k180deg
                : Rotation2d.kCCW_90deg;
    }

    @Override
    public void controlClimb(double speed) {
        _motorSpeed = speed;
    }

    @Override
    public void controlSupport(DoubleSolenoid.Value value) {
        _supportState = value;
    }

    @Override
    public void controlFrictionBrake(DoubleSolenoid.Value value) {
        _frictionBrakeState = value;
    }
}