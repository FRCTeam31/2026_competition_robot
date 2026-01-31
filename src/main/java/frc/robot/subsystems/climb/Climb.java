package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;

public class Climb extends SubsystemBase {
    private IClimb _climb;

    public enum ClimbState {
        Up,
        Stopped,
        Down
    }

    public enum SupportState {
        Raised,
        Lowered
    }

    public enum FrictionBrakeState {
        Applied,
        Released
    }

    public Climb(boolean isReal) {
        _climb = isReal ? new ClimbReal() : new ClimbSim();
    }

    private void actOnState(ClimbInputsAutoLogged inputs) {
        switch (inputs.climbState) {
            case Up:
                _climb.controlClimb(inputs.upperLimitSwitch ? 0 : 0.5);
                break;
            case Down:
                _climb.controlClimb(inputs.lowerLimitSwitch ? 0 : -0.5);
                break;
            case Stopped:
            default:
                _climb.controlClimb(0);
                break;
        }

        _climb.controlSupport(inputs.supportState == SupportState.Raised
                ? DoubleSolenoid.Value.kReverse
                : DoubleSolenoid.Value.kForward);

        _climb.controlFrictionBrake(inputs.frictionBrakeState == FrictionBrakeState.Applied
                ? DoubleSolenoid.Value.kForward
                : DoubleSolenoid.Value.kReverse);

    }

    @Override
    public void periodic() {
        _climb.updateInputs(SuperStructure.Climb);

        actOnState(SuperStructure.Climb);
    }
}
