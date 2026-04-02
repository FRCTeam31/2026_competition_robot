package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;

@AutoLog
public class HopperInputs {

    public Hopper.TransferFeedState TransferFeedState = Hopper.TransferFeedState.STOPPED;
    public Hopper.HopperIntakeState IntakeControlState = Hopper.HopperIntakeState.IN;
    public Hopper.IntakeFeedState IntakeFeedState = Hopper.IntakeFeedState.STOPPED;
    public boolean IntakeIn = IntakeControlState == HopperIntakeState.IN;
}
