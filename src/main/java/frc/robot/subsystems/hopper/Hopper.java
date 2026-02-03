package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SuperStructure;

public class Hopper extends SubsystemBase {
    private IHopper _hopper;

    private Trigger _pulseHopperTrigger = new Trigger(() -> SuperStructure.Hopper.hopperState == HopperState.PULSING)
            .whileTrue(pulseHopperPrivateCommand());

    public enum HopperState { // For extending and retracting Hopper
        IN,
        OUT,
        OFF,
        PULSING
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
        OUT,
        OFF
    }

    public Hopper(boolean isReal) {
        setName("Hopper");

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

        // Intake solenoid control
        switch (inputs.intakeControlState) {
            case IN:
            default:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kReverse);
                break;
            case OUT:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kForward);
                break;
            case OFF:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kOff);
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

        // Hopper solenoid control
        switch (inputs.hopperState) {
            case IN:
            default:
                _hopper.setHopper(DoubleSolenoid.Value.kReverse);
                break;
            case OUT:
                _hopper.setHopper(DoubleSolenoid.Value.kForward);
                break;
            case OFF:
                _hopper.setHopper(DoubleSolenoid.Value.kOff);
                break;
        }
    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);
        Logger.processInputs(getName(), SuperStructure.Hopper);

        actOnState(SuperStructure.Hopper);
    }

    // Hopper Commands

    private Command pulseHopperPrivateCommand() {
        return this.run(() -> _hopper.setHopper(DoubleSolenoid.Value.kReverse))
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
                .andThen(() -> _hopper.setHopper(DoubleSolenoid.Value.kForward))
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay));
    }

    public Command setHopperPulse() {
        return this.runOnce(() -> SuperStructure.Hopper.hopperState = HopperState.PULSING);
    }

    public Command setHopperOut() {
        return this.runOnce(() -> SuperStructure.Hopper.hopperState = HopperState.OUT);
    }

    public Command setHopperIn() {
        return this.runOnce(() -> SuperStructure.Hopper.hopperState = HopperState.IN);
    }

    public Command setHopperOff() {
        return this.runOnce(() -> SuperStructure.Hopper.hopperState = HopperState.OFF);
    }

    // Feed Commands

    public Command setFeedInwards() {
        return this.runOnce(() -> SuperStructure.Hopper.feedState = FeedState.INWARDS);
    }

    public Command setFeedOutwards() {
        return this.runOnce(() -> SuperStructure.Hopper.feedState = FeedState.OUTWARDS);
    }

    public Command stopFeed() {
        return this.runOnce(() -> SuperStructure.Hopper.feedState = FeedState.STOPPED);
    }

    // Intake Feed Commands

    public Command setIntakeFeedInwards() {
        return this.runOnce(() -> SuperStructure.Hopper.intakeFeedState = IntakeFeedState.INWARDS);
    }

    public Command setIntakeFeedOutwards() {
        return this.runOnce(() -> SuperStructure.Hopper.intakeFeedState = IntakeFeedState.OUTWARDS);
    }

    public Command stopIntakeFeed() {
        return this.runOnce(() -> SuperStructure.Hopper.intakeFeedState = IntakeFeedState.STOPPED);
    }

    // Intake Control State

    public Command setIntakeIn() {
        return this.runOnce(() -> SuperStructure.Hopper.intakeControlState = IntakeControlState.IN);
    }

    public Command setIntakeOut() {
        return this.runOnce(() -> SuperStructure.Hopper.intakeControlState = IntakeControlState.OUT);
    }

    public Command setIntakeOff() {
        return this.runOnce(() -> SuperStructure.Hopper.intakeControlState = IntakeControlState.OFF);
    }
}
