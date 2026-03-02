package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.Climb.ClimbControlState;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb.FrictionBrakeState;
import frc.robot.subsystems.climb.Climb.SupportState;

/** AdvantageKit-logged inputs for the Climb subsystem. */
@AutoLog
public class ClimbInputs {

    // States
    public ClimbState ClimbState = Climb.ClimbState.STOPPED;
    public SupportState SupportState = Climb.SupportState.RAISED;
    public FrictionBrakeState FrictionBrakeState = Climb.FrictionBrakeState.RELEASED;
    public ClimbControlState ClimbControlState = Climb.ClimbControlState.RESET;

    // Readings
    public boolean LowerLimitSwitch = true;
    public double MotorRotations = 0;
}
