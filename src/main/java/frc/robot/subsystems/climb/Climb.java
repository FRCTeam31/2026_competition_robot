package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Meters;

import org.prime.subsystems.LoggedSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.Robot;
import frc.robot.SuperStructure;

public class Climb extends LoggedSubsystem {
    private IClimb _climb;

    public enum ClimbState {
        UP,
        STOPPED,
        DOWN
    }

    public enum SupportState {
        RAISED,
        LOWERED
    }

    public enum FrictionBrakeState {
        APPLIED,
        RELEASED
    }

    /**
     * Represents the current state in the
     * climbing process used to restrict climb
     * commands from running out of order or
     * at the same time
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

    public enum ClimbInversionState {
        REGULAR_FUNCTION,
        HAS_BECOME_INVERTED
    }

    // BooleanEvents
    private BooleanEvent _limitSwitchRising;
    private BooleanEvent _climbAtSetpointRising;

    public Climb() {
        setName("Climb");
        _climb = Robot.isReal()
                ? new ClimbReal()
                : new ClimbSim();

        setupLimitSwitchBooleanEvent();
        setupClimbAtSetpointBooleanEvent();
    }

    private void setupLimitSwitchBooleanEvent() {
        // May need to add a short debounce to this if we notice the limit switch getting untriggered,
        // then immediately triggered again when down
        _limitSwitchRising = new BooleanEvent(Robot.EventLoop, () -> SuperStructure.Climb.loweredLimitSwitch).rising();

        // Triggers when the limit switch goes from false (unpressed) to true (pressed)
        // Prevents the climber from being stopped when moving away from the limit switch or ignoring the limit switch when
        // unexpectedly moving inversely of commanded
        _limitSwitchRising.ifHigh(() -> {
            // Stop the climber
            SuperStructure.Climb.climbState = ClimbState.STOPPED;
            _climb.controlClimb(0);

            var climbingUp = SuperStructure.Climb.climbState == ClimbState.UP;
            var climbingDown = SuperStructure.Climb.climbState == ClimbState.DOWN;

            var regularFunction = SuperStructure.Climb.climbInversionState == ClimbInversionState.REGULAR_FUNCTION;
            var invertedFunction = SuperStructure.Climb.climbInversionState == ClimbInversionState.HAS_BECOME_INVERTED;

            // The limit switch has become pressed when the climb should be moving upwards
            // Has become inverted
            var climbingUpWrongly = climbingUp && regularFunction;

            // The limit switch has become pressed when the climb was thought to be inverted and mechanically moving upwards (DOWN state)
            // Return to regular function
            var climbingDownWrongly = climbingDown && invertedFunction;

            if (climbingUpWrongly) {
                SuperStructure.Climb.climbInversionState = ClimbInversionState.HAS_BECOME_INVERTED;
            } else if (climbingDownWrongly) {
                SuperStructure.Climb.climbInversionState = ClimbInversionState.REGULAR_FUNCTION;
            }
        });
    }

    private void setupClimbAtSetpointBooleanEvent() {
        // Check how far the climb is from maximum extension, triggering if within a preset range
        _climbAtSetpointRising = new BooleanEvent(Robot.EventLoop, () -> {
            var error = ClimbMap.MAX_CLIMB_EXTENSION.in(Meters) - SuperStructure.Climb.climberExtension.in(Meters);
            return Math.abs(error) <= ClimbMap.CLIMB_AT_SETPOINT_ERROR.in(Meters);
        });

        // Stop the climber if near the max extension, will be a lot faster to tune than PID and should be reliable enough
        _climbAtSetpointRising.ifHigh(() -> {
            SuperStructure.Climb.climbState = ClimbState.STOPPED;
            _climb.controlClimb(0);
        });
    }

    private void actOnState(ClimbInputsAutoLogged inputs) {

        var inversionCorrectionSign = inputs.climbInversionState == ClimbInversionState.HAS_BECOME_INVERTED ? -1 : 1;

        var climbingUp = inputs.climbState == ClimbState.UP;
        var climbingDown = inputs.climbState == ClimbState.DOWN;

        var regularFunction = inputs.climbInversionState == ClimbInversionState.REGULAR_FUNCTION;
        var invertedFunction = inputs.climbInversionState == ClimbInversionState.HAS_BECOME_INVERTED;

        var limitSwitchPressed = inputs.loweredLimitSwitch;

        var climbingDownWrongly = climbingDown || limitSwitchPressed || regularFunction;
        var climbingUpWrongly = climbingUp || limitSwitchPressed || invertedFunction;

        if (climbingDownWrongly || climbingUpWrongly) {
            SuperStructure.Climb.climbState = ClimbState.STOPPED;
            _climb.controlClimb(0);

            // Ensure the friction brake is not applied before moving the climber
        } else if (inputs.frictionBrakeState == FrictionBrakeState.RELEASED) {
            switch (inputs.climbState) {
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
            SuperStructure.Climb.climbState = ClimbState.STOPPED;
            _climb.controlClimb(0);
        }

        // Zero the encoder when the limit switch is pressed, regardless of climb states or if the value is rising or falling
        if (limitSwitchPressed) {
            _climb.zeroEncoder();
        }

        _climb.controlSupport(inputs.supportState == SupportState.RAISED
                ? DoubleSolenoid.Value.kReverse
                : DoubleSolenoid.Value.kForward);

        _climb.controlFrictionBrake(inputs.frictionBrakeState == FrictionBrakeState.APPLIED
                ? DoubleSolenoid.Value.kForward
                : DoubleSolenoid.Value.kReverse);

    }

    @Override
    public void periodic() {
        _climb.updateInputs(SuperStructure.Climb);
        processInputs(SuperStructure.Climb);

        actOnState(SuperStructure.Climb);
    }

    // #region Commands

    /**
     * Sets the climb motor state
     * @param state The desired climb state (UP, DOWN, STOPPED)
     * @return Command to set the state
     */
    public Command setClimb(ClimbState state) {
        return this.runOnce(() -> SuperStructure.Climb.climbState = state);
    }

    /**
     * Sets the support solenoid state
     * @param state The desired support state (RAISED, LOWERED)
     * @return Command to set the state
     */
    public Command setSupport(SupportState state) {
        return this.runOnce(() -> SuperStructure.Climb.supportState = state);
    }

    /**
     * Sets the friction brake state
     * @param state The desired brake state (APPLIED, RELEASED)
     * @return Command to set the state
     */
    public Command setBrake(FrictionBrakeState state) {
        return this.runOnce(() -> SuperStructure.Climb.frictionBrakeState = state);
    }

    // #endregion
}
