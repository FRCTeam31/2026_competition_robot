package frc.robot.subsystems.climb;

import org.prime.subsystems.LoggedSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SuperStructure;

/**
 * Climb subsystem responsible for raising and lowering the robot's climbing mechanism.
 * Uses MAXMotion position control for the climb motor, and manages the support
 * solenoid and friction brake solenoid.
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
     * Used by {@link frc.robot.Container} to restrict climb commands from running out of order.
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

    private IClimb _climb;

    public Climb() {
        setName("Climb");
        _climb = Robot.isReal()
                ? new ClimbReal()
                : new ClimbSim();
    }

    // #region Private Methods

    /**
     * Acts on the current climb state to control the motor, support solenoid, and friction brake.
     * The motor uses MAXMotion position control — UP goes to full extension, DOWN goes to home.
     *
     * @param inputs The current climb inputs from {@link SuperStructure}
     */
    private void actOnState(ClimbInputsAutoLogged inputs) {
        // Zero the encoder when the limit switch is pressed, regardless of climb state
        if (inputs.LowerLimitSwitch) {
            _climb.zeroEncoder();
        }

        // Motor control
        var brakeReleased = inputs.FrictionBrakeState == FrictionBrakeState.RELEASED;

        if (!brakeReleased) {
            // Never drive the motor while the brake is applied
            _climb.stopClimb();
        } else if (inputs.ClimbState == ClimbState.UP) {
            _climb.setClimbPosition(ClimbMap.EXTENDED_ROTATIONS);
        } else if (inputs.ClimbState == ClimbState.DOWN && !inputs.LowerLimitSwitch) {
            // Only allow downward movement if the limit switch is not pressed
            _climb.setClimbPosition(ClimbMap.RETRACTED_ROTATIONS);
        } else {
            _climb.stopClimb();
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
