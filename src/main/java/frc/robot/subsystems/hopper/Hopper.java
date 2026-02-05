package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SuperStructure;

public class Hopper extends SubsystemBase {
    private IHopper _hopper;

    @SuppressWarnings("unused")
    private Trigger _pulseHopperTrigger = new Trigger(
            () -> SuperStructure.Hopper.ExtensionState == ExtensionState.PULSING)
            .whileTrue(pulseHopperPrivateCommand());

    public enum ExtensionState { // For extending and retracting Hopper
        IN,
        OUT,
        OFF,
        PULSING
    }

    public enum TransferFeedState { // For feeding in and out to the shooter
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
        switch (inputs.TransferFeedState) {
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
        switch (inputs.IntakeControlState) {
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
        switch (inputs.IntakeFeedState) {
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
        switch (inputs.ExtensionState) {
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

    // #region Commands

    private Command pulseHopperPrivateCommand() {
        return this.run(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.IN)
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
                .andThen(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.OUT)
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay));
    }

    /**
     * Sets the hopper solenoid state
     * @param state The desired hopper state (IN, OUT, OFF, PULSING)
     * @return Command to set the state
     */
    public Command setHopper(ExtensionState state) {
        return this.runOnce(() -> SuperStructure.Hopper.ExtensionState = state);
    }

    /**
     * Sets the transfer feed motor state
     * @param state The desired feed state (INWARDS, OUTWARDS, STOPPED)
     * @return Command to set the state
     */
    public Command setFeed(TransferFeedState state) {
        return this.runOnce(() -> SuperStructure.Hopper.TransferFeedState = state);
    }

    /**
     * Sets the intake feed motor state
     * @param state The desired intake feed state (INWARDS, OUTWARDS, STOPPED)
     * @return Command to set the state
     */
    public Command setIntakeFeed(IntakeFeedState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeFeedState = state);
    }

    /**
     * Sets the intake solenoid control state
     * @param state The desired intake control state (IN, OUT, OFF)
     * @return Command to set the state
     */
    public Command setIntakeControl(IntakeControlState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = state);
    }

    // #endregion
}
