package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HopperInputs {
    // States
    public Hopper.ExtensionState ExtensionState = Hopper.ExtensionState.IN;
    public Hopper.TransferFeedState TransferFeedState = Hopper.TransferFeedState.STOPPED;
    public Hopper.HopperIntakeState IntakeControlState = Hopper.HopperIntakeState.IN;
    public Hopper.IntakeFeedState IntakeFeedState = Hopper.IntakeFeedState.STOPPED;

    // Mechanism Poses
    public Pose3d hopperComponentPose = new Pose3d();
    public Pose3d intakeComponentPose = new Pose3d();
    public Pose3d intakeFeedComponentPose = new Pose3d();
    public Pose3d topFeedBarComponentPose = new Pose3d();
    public Pose3d bottomFeedBarComponentPose = new Pose3d();

    // Feed Positions
    public double intakeFeedPosition;
    public double hopperFeedPosition;
}
