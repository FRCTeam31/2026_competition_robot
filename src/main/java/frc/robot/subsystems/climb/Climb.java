package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SuperStructure;

public class Climb extends SubsystemBase {
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

    public Climb() {
        setName("Climb");
        _climb = Robot.isReal()
                ? new ClimbReal()
                : new ClimbSim();
    }

    private void actOnState(ClimbInputsAutoLogged inputs) {
        switch (inputs.climbState) {
            case UP:
                _climb.controlClimb(inputs.upperLimitSwitch ? 0 : 0.5);
                break;
            case DOWN:
                _climb.controlClimb(inputs.lowerLimitSwitch ? 0 : -0.5);
                break;
            case STOPPED:
            default:
                _climb.controlClimb(0);
                break;
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
        Logger.processInputs(getName(), SuperStructure.Climb);

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
