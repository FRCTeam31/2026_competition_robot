package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimbInputs {
    // States
    public Climb.ClimbState climbState = Climb.ClimbState.STOPPED;
    public Climb.SupportState supportState = Climb.SupportState.RAISED;
    public Climb.FrictionBrakeState frictionBrakeState = Climb.FrictionBrakeState.RELEASED;
    public Climb.ClimbControlState climbControlState = Climb.ClimbControlState.RESET;

    // Limit Switches
    public boolean upperLimitSwitch = false;
    public boolean lowerLimitSwitch = true;

    // Mechanism Movement
    public double climbExtension = 0;
    public Rotation2d supportAngle = new Rotation2d();

    // Mechanism Poses
    public Pose3d climbComponentPose = Pose3d.kZero;
    public Pose3d supportComponentPose = Pose3d.kZero;
}