package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public Climb(boolean isReal) {
        setName("Climb");
        _climb = isReal ? new ClimbReal() : new ClimbSim();
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

    // Climb Commands

    public Command setClimbUp() {
        return this.runOnce(() -> SuperStructure.Climb.climbState = ClimbState.UP);
    }

    public Command setClimbDown() {
        return this.runOnce(() -> SuperStructure.Climb.climbState = ClimbState.DOWN);
    }

    public Command stopClimb() {
        return this.runOnce(() -> SuperStructure.Climb.climbState = ClimbState.STOPPED);
    }

    // Support Commands

    public Command setSupportRaised() {
        return this.runOnce(() -> SuperStructure.Climb.supportState = SupportState.RAISED);
    }

    public Command setSupportLowered() {
        return this.runOnce(() -> SuperStructure.Climb.supportState = SupportState.LOWERED);
    }

    // Friction Brake Commands

    public Command setBrakeApplied() {
        return this.runOnce(() -> SuperStructure.Climb.frictionBrakeState = FrictionBrakeState.APPLIED);
    }

    public Command setBrakeReleased() {
        return this.runOnce(() -> SuperStructure.Climb.frictionBrakeState = FrictionBrakeState.RELEASED);
    }
}
