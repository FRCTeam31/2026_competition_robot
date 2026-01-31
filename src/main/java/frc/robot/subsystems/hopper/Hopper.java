package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SuperStructure;

public class Hopper extends SubsystemBase {
    private IHopper _hopper;
    private DoubleSolenoid.Value _lastHopperValue;

    public enum HopperPosition { // For extending and retracting Hopper
        IN,
        OUT
    }

    public enum FeedState { // For feeding in and out to the shooter
        INWARDS,
        OUTWARDS,
        STOPPED
    }

    public enum IntakeFeedState { // For intake wheels
        INWARDS,
        OUTWARDS,
        STOPPED
    }

    public enum IntakeControlState { // For intake rotation
        IN,
        OUT
    }

    public Hopper(boolean isReal) {
        _hopper = isReal ? new HopperReal() : new HopperSim();
    }

    private void actOnState(HopperInputsAutoLogged inputs) {
        // Feed motor control
        switch (inputs.feedState) {
            case INWARDS:
                _hopper.setFeedSpeed(0.5);
                break;
            case OUTWARDS:
                _hopper.setFeedSpeed(-0.5);
                break;
            case STOPPED:
            default:
                _hopper.setFeedSpeed(0);
                break;
        }

        // Solenoid control
        switch (inputs.intakeControlState) {
            case OUT:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kForward);
                break;
            case IN:
            default:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kReverse);
                break;
        }

        // Intake feed motor control
        switch (inputs.intakeFeedState) {
            case INWARDS:
                _hopper.setIntakeFeedSpeed(.5);
                break;
            case OUTWARDS:
                _hopper.setIntakeFeedSpeed(-.5);
                break;
            case STOPPED:
            default:
                _hopper.stopIntake();
                break;
        }
    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);

        actOnState(SuperStructure.Hopper);
    }

    public Command setHopperOut() {
        return this.runOnce(() -> {
            _hopper.setHopper(DoubleSolenoid.Value.kForward);
            _lastHopperValue = DoubleSolenoid.Value.kForward;
        });
    }

    public Command setHopperIn() {
        return this.runOnce(() -> {
            _hopper.setHopper(DoubleSolenoid.Value.kReverse);
            _lastHopperValue = DoubleSolenoid.Value.kReverse;
        });
    }

    public Command pulseHopper() {
        return this.run(() -> _hopper.toggleHopper())
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
                .finallyDo(() -> _hopper.setHopper(_lastHopperValue));
    }
}
