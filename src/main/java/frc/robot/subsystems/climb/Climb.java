package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;

import org.prime.subsystems.LoggedSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SuperStructure;

/**
 * Climb subsystem responsible for raising and lowering the robot's climbing mechanism.
 * Manages the climb motor, support solenoid, and friction brake solenoid.
 * Uses {@link BooleanEvent}s to handle limit switch and setpoint safety stops.
 */
public class Climb extends LoggedSubsystem {

    // #region Enums

    /** Represents the desired direction of the climb motor. */
    public enum ClimbState {
        UP,
        STOPPED,
        DOWN
    }

    /** Represents the desired state of the support solenoid. */
    public enum SupportState {
        RAISED,
        LOWERED
    }

    /** Represents the desired state of the friction brake solenoid. */
    public enum FrictionBrakeState {
        APPLIED,
        RELEASED
    }

    /**
     * Represents the current state in the climbing process.
     * Used to restrict climb commands from running out of order or at the same time.
     */
    public enum ClimbControlState {
        RESET,
        RESETTING,
        SETUP_IN_PROGRESS,
        SETUP_DONE,
        CLIMBING_UP,
        HAS_CLIMBED,
        CLIMBING_DOWN,
        CLIMBING_DONE
    }

    // #endregion

    // #region Fields

    private IClimb _climb;
    private BooleanEvent _limitSwitchRising;
    private BooleanEvent _climbAtSetpointRising;

    // #endregion

    // #region Constructor

    public Climb() {
        setName("Climb");
        _climb = Robot.isReal()
                ? new ClimbReal()
                : new ClimbSim();

        setupLimitSwitchBooleanEvent();
        setupClimbAtSetpointBooleanEvent();
    }

    // #endregion

    // #region Private Methods

    /**
     * Stops the climb motor and sets the climb state to STOPPED.
     */
    private void stopClimber() {
        SuperStructure.Climb.ClimbState = ClimbState.STOPPED;
        _climb.controlClimb(0);
    }

    /**
     * Returns whether the climber is moving in the wrong direction based on
     * the limit switch state and inversion tracking.
     * <p>
     * "Wrong direction" means the limit switch is pressed while the climber is
     * mechanically moving towards it (downward). This accounts for motor inversion.
     *
     * @param state     The current climb direction
     * @param isInverted Whether the motor is currently tracked as inverted
     * @param limitPressed Whether the lower limit switch is pressed
     * @return true if the climber should be stopped
     */
    private boolean isMovingWrongDirection(ClimbState state, boolean isInverted, boolean limitPressed) {
        if (!limitPressed)
            return false;

        // In regular function: DOWN moves toward limit switch
        // When inverted: UP moves toward limit switch (motor is wired backwards)
        var movingDownNormally = (state == ClimbState.DOWN) && !isInverted;
        var movingDownInverted = (state == ClimbState.UP) && isInverted;

        return movingDownNormally || movingDownInverted;
    }

    /**
     * Calculates the motor output percent for the given climb state, accounting
     * for the inversion flag and the motor inversion constant.
     *
     * @param state      The desired climb direction
     * @param isInverted Whether the motor is currently tracked as inverted
     * @return Motor percent output in [-1, 1], or 0 for STOPPED
     */
    private double getMotorOutput(ClimbState state, boolean isInverted) {
        var directionSign = switch (state) {
            case UP -> 1;
            case DOWN -> -1;
            case STOPPED -> 0;
        };

        var inversionSign = isInverted ? -1 : 1;
        var motorInversionSign = ClimbMap.CLIMB_MOTOR_INVERTED ? -1 : 1;

        return ClimbMap.MAX_CLIMB_MOTOR_PERCENT_OUT * directionSign * inversionSign * motorInversionSign;
    }

    /**
     * Configures the limit switch {@link BooleanEvent} to stop the climber and
     * detect motor inversion when the limit switch transitions from unpressed to pressed.
     */
    private void setupLimitSwitchBooleanEvent() {
        // May need to add a short debounce to this if we notice the limit switch getting untriggered,
        // then immediately triggered again when down
        _limitSwitchRising = new BooleanEvent(Robot.EventLoop, () -> SuperStructure.Climb.LowerLimitSwitch).rising();

        // Triggers when the limit switch goes from false (unpressed) to true (pressed).
        // Detects if the motor direction is inverted relative to commanded direction.
        _limitSwitchRising.ifHigh(() -> {
            // Capture the current state before stopping, so we can detect inversion
            var wasMovingUp = SuperStructure.Climb.ClimbState == ClimbState.UP;
            var wasMovingDown = SuperStructure.Climb.ClimbState == ClimbState.DOWN;
            var wasInverted = SuperStructure.Climb.IsInverted;

            stopClimber();

            // The limit switch is at the bottom. If it triggers while commanding UP
            // (in regular function), the motor must be inverted. Vice versa for recovery.
            if (wasMovingUp && !wasInverted) {
                SuperStructure.Climb.IsInverted = true;
            } else if (wasMovingDown && wasInverted) {
                SuperStructure.Climb.IsInverted = false;
            }
        });
    }

    /**
     * Configures the setpoint {@link BooleanEvent} to stop the climber when
     * the extension is within a preset range of the maximum extension.
     */
    private void setupClimbAtSetpointBooleanEvent() {
        _climbAtSetpointRising = new BooleanEvent(Robot.EventLoop, () -> {
            var error = ClimbMap.MAX_CLIMB_EXTENSION.in(Meters) - SuperStructure.Climb.DistanceExtended.in(Meters);
            return Math.abs(error) <= ClimbMap.CLIMB_AT_SETPOINT_ERROR.in(Meters);
        });

        _climbAtSetpointRising.ifHigh(this::stopClimber);
    }

    /**
     * Acts on the current climb state to control the motor, support solenoid, and friction brake.
     * Handles inversion correction, limit switch safety, and encoder zeroing.
     *
     * @param inputs The current climb inputs from {@link SuperStructure}
     */
    private void actOnState(ClimbInputsAutoLogged inputs) {
        // Safety: stop if moving toward the pressed limit switch
        if (isMovingWrongDirection(inputs.ClimbState, inputs.IsInverted, inputs.LowerLimitSwitch)) {
            stopClimber();
        } else if (inputs.FrictionBrakeState == FrictionBrakeState.RELEASED) {
            // Only allow motor movement when the friction brake is released
            _climb.controlClimb(getMotorOutput(inputs.ClimbState, inputs.IsInverted));
        } else {
            // Brake is applied or state is unexpected — ensure motor is stopped
            stopClimber();
        }

        // Zero the encoder when the limit switch is pressed, regardless of climb state
        if (inputs.LowerLimitSwitch) {
            _climb.zeroEncoder();
        }

        // Solenoid control
        _climb.controlSupport(inputs.SupportState == SupportState.RAISED
                ? DoubleSolenoid.Value.kReverse
                : DoubleSolenoid.Value.kForward);

        _climb.controlFrictionBrake(inputs.FrictionBrakeState == FrictionBrakeState.APPLIED
                ? DoubleSolenoid.Value.kForward
                : DoubleSolenoid.Value.kReverse);
    }

    // #endregion

    // #region Periodic

    @Override
    public void periodic() {
        _climb.updateInputs(SuperStructure.Climb);
        processInputs(SuperStructure.Climb);

        actOnState(SuperStructure.Climb);
    }

    // #endregion

    // #region Commands

    /**
     * Sets the climb motor state.
     *
     * @param state The desired climb state (UP, DOWN, STOPPED)
     * @return Command to set the state
     */
    public Command setClimb(ClimbState state) {
        return this.runOnce(() -> SuperStructure.Climb.ClimbState = state);
    }

    /**
     * Sets the support solenoid state.
     *
     * @param state The desired support state (RAISED, LOWERED)
     * @return Command to set the state
     */
    public Command setSupport(SupportState state) {
        return this.runOnce(() -> SuperStructure.Climb.SupportState = state);
    }

    /**
     * Sets the friction brake state.
     *
     * @param state The desired brake state (APPLIED, RELEASED)
     * @return Command to set the state
     */
    public Command setBrake(FrictionBrakeState state) {
        return this.runOnce(() -> SuperStructure.Climb.FrictionBrakeState = state);
    }

    // #endregion
}
