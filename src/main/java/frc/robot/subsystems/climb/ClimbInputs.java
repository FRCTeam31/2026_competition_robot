package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.Climb.ClimbControlState;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb.FrictionBrakeState;
import frc.robot.subsystems.climb.Climb.SupportState;

@AutoLog
public class ClimbInputs {
    public Climb.ClimbState climbState = ClimbState.STOPPED;
    public Climb.SupportState supportState = SupportState.RAISED;
    public Climb.FrictionBrakeState frictionBrakeState = FrictionBrakeState.RELEASED;
    public Climb.ClimbControlState climbControlState = ClimbControlState.RESET;
    public boolean upperLimitSwitch = false;
    public boolean lowerLimitSwitch = true;
}
