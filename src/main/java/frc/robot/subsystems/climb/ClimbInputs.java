package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb.FrictionBrakeState;
import frc.robot.subsystems.climb.Climb.SupportState;

@AutoLog
public class ClimbInputs {
    public Climb.ClimbState climbState = ClimbState.Stopped;
    public Climb.SupportState supportState = SupportState.Raised;
    public Climb.FrictionBrakeState frictionBrakeState = FrictionBrakeState.Released;
    public boolean upperLimitSwitch = false;
    public boolean lowerLimitSwitch = true;
}
