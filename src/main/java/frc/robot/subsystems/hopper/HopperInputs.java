package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HopperInputs {

    public Hopper.HopperPosition hopperPosition;
    public Hopper.FeedState feedState;
    public Hopper.IntakeControlState intakeControlState;
    public Hopper.IntakeFeedState intakeFeedState;
    public boolean intakeINLimitSwitch;
    public boolean intakeOUTLimitSwitch;
}
