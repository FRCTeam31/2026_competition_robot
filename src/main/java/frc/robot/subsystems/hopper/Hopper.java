package frc.robot.subsystems.hopper;

import java.util.function.Supplier;

import org.prime.subsystems.LoggedSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;

public class Hopper extends LoggedSubsystem {
    private IHopper _hopper;

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

    private final double _intakeFeedInvertedSign = HopperMap.INTAKE_FEED_INVERTED ? -1 : 1;
    private double _intakeFeedPercentOut = HopperMap.MAX_INTAKE_FEED_PERCENT_OUT;
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
                _hopper.setIntakeFeedSpeed(_intakeFeedPercentOut * _intakeFeedInvertedSign);
                break;
            case OUTWARDS:
                _hopper.setIntakeFeedSpeed(-_intakeFeedPercentOut * _intakeFeedInvertedSign);
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

    public Command setIntakeFeedSupplier(Supplier<IntakeFeedState> state) {
        return this.run(() -> SuperStructure.Hopper.IntakeFeedState = state.get());
    }

    /**
     * Sets the intake solenoid control state
     * @param state The desired intake control state (IN, OUT, OFF)
     * @return Command to set the state
     */
    public Command setHopperIntakeControl(HopperIntakeState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = state);
    }

    /**
     * Toggles the intake solenoid control state between IN and OUT, and sets the intake feed state accordingly
     */
    public Command toggleHopperIntake() {
        return this.runOnce(() -> {
            SuperStructure.Hopper.IntakeControlState = SuperStructure.Hopper.IntakeControlState == HopperIntakeState.OUT
                    ? HopperIntakeState.IN
                    : HopperIntakeState.OUT;

            SuperStructure.Hopper.IntakeFeedState = SuperStructure.Hopper.IntakeControlState == HopperIntakeState.OUT
                    ? IntakeFeedState.INWARDS
                    : IntakeFeedState.STOPPED;
        });
    }

    public Command overrideIntakeFeedPercentOut(double percentOut) {
        return this.run(() -> {
            overrideIntakeFeedPercentOut(percentOut);
        }).finallyDo(() -> overrideIntakeFeedPercentOut(HopperMap.MAX_INTAKE_FEED_PERCENT_OUT));
    }

    /**
    * Oscillates the intake solenoid between IN and OUT to jostle balls toward the transfer/feed.
    * When interrupted (e.g., firing stops), leaves the intake in the OUT position.
    * @param cycleDurationSeconds The duration of one full IN/OUT cycle
    * @return A command that runs until interrupted
    */
    public Command oscillateIntake(double cycleDurationSeconds) {
        double halfCycle = cycleDurationSeconds / 2.0;
        return Commands.repeatingSequence(
                this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.IN),
                Commands.waitSeconds(halfCycle),
                this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.OUT),
                Commands.waitSeconds(halfCycle))
                .finallyDo(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.OUT);
    }

    // #endregion
}
