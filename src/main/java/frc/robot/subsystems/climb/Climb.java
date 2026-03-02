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

    /** Tracks whether the climb motor direction has become inverted relative to commanded direction. */
    public enum ClimbInversionState {
        REGULAR_FUNCTION,
        HAS_BECOME_INVERTED
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
     * Configures the limit switch {@link BooleanEvent} to stop the climber and
     * detect motor inversion when the limit switch transitions from unpressed to pressed.
     */
    private void setupLimitSwitchBooleanEvent() {
        // May need to add a short debounce to this if we notice the limit switch getting untriggered,
        // then immediately triggered again when down
        _limitSwitchRising = new BooleanEvent(Robot.EventLoop, () -> SuperStructure.Climb.LowerLimitSwitch).rising();

        // Triggers when the limit switch goes from false (unpressed) to true (pressed)
        // Prevents the climber from being stopped when moving away from the limit switch or ignoring the limit switch when
        // unexpectedly moving inversely of commanded
        _limitSwitchRising.ifHigh(() -> {
            // Stop the climber
            SuperStructure.Climb.ClimbState = ClimbState.STOPPED;
            _climb.controlClimb(0);

            var climbingUp = SuperStructure.Climb.ClimbState == ClimbState.UP;
            var climbingDown = SuperStructure.Climb.ClimbState == ClimbState.DOWN;

            var regularFunction = SuperStructure.Climb.ClimbInversionState == ClimbInversionState.REGULAR_FUNCTION;
            var invertedFunction = SuperStructure.Climb.ClimbInversionState == ClimbInversionState.HAS_BECOME_INVERTED;

            // The limit switch has become pressed when the climb should be moving upwards
            // Has become inverted
            var climbingUpWrongly = climbingUp && regularFunction;

            // The limit switch has become pressed when the climb was thought to be inverted and mechanically moving upwards (DOWN state)
            // Return to regular function
            var climbingDownWrongly = climbingDown && invertedFunction;

            if (climbingUpWrongly) {
                SuperStructure.Climb.ClimbInversionState = ClimbInversionState.HAS_BECOME_INVERTED;
            } else if (climbingDownWrongly) {
                SuperStructure.Climb.ClimbInversionState = ClimbInversionState.REGULAR_FUNCTION;
            }
        });
    }

    /**
     * Configures the setpoint {@link BooleanEvent} to stop the climber when
     * the extension is within a preset range of the maximum extension.
     */
    private void setupClimbAtSetpointBooleanEvent() {
        // Check how far the climb is from maximum extension, triggering if within a preset range
        _climbAtSetpointRising = new BooleanEvent(Robot.EventLoop, () -> {
            var error = ClimbMap.MAX_CLIMB_EXTENSION.in(Meters) - SuperStructure.Climb.DistanceExtended.in(Meters);
            return Math.abs(error) <= ClimbMap.CLIMB_AT_SETPOINT_ERROR.in(Meters);
        });

        // Stop the climber if near the max extension, will be a lot faster to tune than PID and should be reliable enough
        _climbAtSetpointRising.ifHigh(() -> {
            SuperStructure.Climb.ClimbState = ClimbState.STOPPED;
            _climb.controlClimb(0);
        });
    }

    /**
     * Acts on the current climb state to control the motor, support solenoid, and friction brake.
     * Handles inversion correction, limit switch safety, and encoder zeroing.
     *
     * @param inputs The current climb inputs from {@link SuperStructure}
     */
    private void actOnState(ClimbInputsAutoLogged inputs) {

        var inversionCorrectionSign = inputs.ClimbInversionState == ClimbInversionState.HAS_BECOME_INVERTED ? -1 : 1;

        var climbingUp = inputs.ClimbState == ClimbState.UP;
        var climbingDown = inputs.ClimbState == ClimbState.DOWN;

        var regularFunction = inputs.ClimbInversionState == ClimbInversionState.REGULAR_FUNCTION;
        var invertedFunction = inputs.ClimbInversionState == ClimbInversionState.HAS_BECOME_INVERTED;

        var limitSwitchPressed = inputs.LowerLimitSwitch;

        var climbingDownWrongly = climbingDown && limitSwitchPressed && regularFunction;
        var climbingUpWrongly = climbingUp && limitSwitchPressed && invertedFunction;

        if (climbingDownWrongly || climbingUpWrongly) {
            SuperStructure.Climb.ClimbState = ClimbState.STOPPED;
            _climb.controlClimb(0);

            // Ensure the friction brake is not applied before moving the climber
        } else if (inputs.FrictionBrakeState == FrictionBrakeState.RELEASED) {
            switch (inputs.ClimbState) {
                case UP:
                    _climb.controlClimb(ClimbMap.MAX_CLIMB_MOTOR_PERCENT_OUT * inversionCorrectionSign
                            * (ClimbMap.CLIMB_MOTOR_INVERTED ? -1 : 1));
                    break;
                case DOWN:
                    _climb.controlClimb(-ClimbMap.MAX_CLIMB_MOTOR_PERCENT_OUT * inversionCorrectionSign
                            * (ClimbMap.CLIMB_MOTOR_INVERTED ? -1 : 1));
                    break;
                case STOPPED:
                default:
                    _climb.controlClimb(0);
                    break;
            }
        } else {
            // Default to a speed of 0 and set the state to STOPPED when above "if" statements are false
            SuperStructure.Climb.ClimbState = ClimbState.STOPPED;
            _climb.controlClimb(0);
        }

        // Zero the encoder when the limit switch is pressed, regardless of climb states or if the value is rising or falling
        if (limitSwitchPressed) {
            _climb.zeroEncoder();
        }

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
