package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HopperInputs {

    public Hopper.ExtensionState ExtensionState = Hopper.ExtensionState.IN;
    public Hopper.TransferFeedState TransferFeedState = Hopper.TransferFeedState.STOPPED;
    public Hopper.IntakeControlState IntakeControlState = Hopper.IntakeControlState.IN;
    public Hopper.IntakeFeedState IntakeFeedState = Hopper.IntakeFeedState.STOPPED;
}
