package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimbInputs {
    public Climb.ClimbState climbState;
    public Climb.SupportState supportState;
    public Climb.FrictionBrakeState frictionBrakeState;
    public boolean upperLimitSwitch;
    public boolean lowerLimitSwitch;
}
