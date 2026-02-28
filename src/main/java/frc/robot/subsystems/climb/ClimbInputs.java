package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.climb.Climb.ClimbControlState;
import frc.robot.subsystems.climb.Climb.ClimbInversionState;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb.FrictionBrakeState;
import frc.robot.subsystems.climb.Climb.SupportState;

@AutoLog
public class ClimbInputs {
    // States
    public Climb.ClimbState climbState = ClimbState.STOPPED;
    public Climb.SupportState supportState = SupportState.RAISED;
    public Climb.FrictionBrakeState frictionBrakeState = FrictionBrakeState.RELEASED;
    public Climb.ClimbControlState climbControlState = ClimbControlState.RESET;
    public Climb.ClimbInversionState climbInversionState = ClimbInversionState.REGULAR_FUNCTION;

    // Readings
    public boolean loweredLimitSwitch = true;
    public Distance climberExtension = Distance.ofBaseUnits(0, Meters);
}
