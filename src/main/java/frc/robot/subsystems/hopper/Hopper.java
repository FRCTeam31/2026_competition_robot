package frc.robot.subsystems.hopper;

import org.prime.subsystems.LoggedSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SuperStructure;

public class Hopper extends LoggedSubsystem {
    private IHopper _hopper;

    @SuppressWarnings("unused")

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

    public enum HopperIntakeState { // For intake rotation
        IN,
        OUT,
        OFF
    }

    private double maxIntakeFeedInwardsPercentOut = (HopperMap.INTAKE_FEED_INVERTED ? -1 : 1)
            * HopperMap.MAX_INTAKE_FEED_PERCENT_OUT;
    private double maxFeedInwardsPercentOut = (HopperMap.FEED_INVERTED ? -1 : 1) * HopperMap.MAX_FEED_PERCENT_OUT;

    public Hopper() {
        setName("Hopper");

        _hopper = Robot.isReal()
                ? new HopperReal()
                : new HopperSim();
    }

    private void actOnState(HopperInputsAutoLogged inputs) {
        // Feed motor control
        switch (inputs.TransferFeedState) {
            case INWARDS:
                _hopper.setFeedSpeed(maxFeedInwardsPercentOut);
                break;
            case OUTWARDS:
                _hopper.setFeedSpeed(-maxFeedInwardsPercentOut);
                break;
            case STOPPED:
            default:
                _hopper.setFeedSpeed(0);
                break;
        }

        // Intake and Hopper solenoid control
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
                _hopper.setIntakeFeedSpeed(maxIntakeFeedInwardsPercentOut);
                break;
            case OUTWARDS:
                _hopper.setIntakeFeedSpeed(-maxIntakeFeedInwardsPercentOut);
                break;
            case STOPPED:
            default:
                _hopper.setIntakeFeedSpeed(0);
                break;
        }

    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);
        processInputs(SuperStructure.Hopper);

        actOnState(SuperStructure.Hopper);
    }

    // #region Commands

    // private Command pulseHopperPrivateCommand() {
    //     return this.run(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.IN)
    //             .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
    //             .andThen(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.OUT)
    //             .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay));
    // }

    /**
     * Sets the hopper solenoid state
     * @param state The desired hopper state (IN, OUT, OFF, PULSING)
     * @return Command to set the state
     */

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
    public Command setHopperIntakeControl(HopperIntakeState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = state);
    }

    // #endregion
}
