package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.hopper.Hopper.FeedState;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.hopper.Hopper.IntakeControlState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;

@AutoLog
public class HopperInputs {

    public Hopper.HopperState hopperState = HopperState.IN;
    public Hopper.FeedState feedState = FeedState.STOPPED;
    public Hopper.IntakeControlState intakeControlState = IntakeControlState.IN;
    public Hopper.IntakeFeedState intakeFeedState = IntakeFeedState.STOPPED;
}
