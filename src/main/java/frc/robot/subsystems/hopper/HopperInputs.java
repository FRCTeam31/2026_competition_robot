package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HopperInputs {
    // States
    public Hopper.TransferFeedState TransferFeedState = Hopper.TransferFeedState.STOPPED;
    public Hopper.HopperIntakeState IntakeControlState = Hopper.HopperIntakeState.IN;
    public Hopper.IntakeFeedState IntakeFeedState = Hopper.IntakeFeedState.STOPPED;

    // Mechanism Poses
    public Pose3d hopperComponentPose = Pose3d.kZero;
    public Pose3d intakeComponentPose = Pose3d.kZero;
    public Pose3d intakeFeedComponentPose = Pose3d.kZero;
    public Pose3d topFeedBarComponentPose = Pose3d.kZero;
    public Pose3d bottomFeedBarComponentPose = Pose3d.kZero;

    // Feed Positions
    public double intakeFeedPosition;
    public double hopperFeedPosition;
}
