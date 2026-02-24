package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HopperInputs {

    public Hopper.TransferFeedState TransferFeedState = Hopper.TransferFeedState.STOPPED;
    public Hopper.HopperIntakeState IntakeControlState = Hopper.HopperIntakeState.IN;
    public Hopper.IntakeFeedState IntakeFeedState = Hopper.IntakeFeedState.STOPPED;
}
