package frc.robot.subsystems.vision.limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.limelight.helpers.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.limelight.helpers.LimelightHelpers.PoseEstimate;

public class LimelightCameraInputs {

    /**
     * The JSON dump from this limelight.
     */
    public LimelightResults CurrentResults = new LimelightResults();

    public boolean TargetValid = false;

    /**
     * Horizontal Offset From Crosshair To Target 
     * (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    public Rotation2d TargetHorizontalOffset = new Rotation2d(Math.PI);

    /**
     * Vertical Offset From Crosshair To Target 
     * (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    public Rotation2d TargetVerticalOffset = new Rotation2d();

    /**
     * The robot's MT1 pose in field space.
     */
    public PoseEstimate BotPoseEstimate = new PoseEstimate();

    /**
     * The pose of the target relative to the robot, as calculated from the limelight's horizontal and vertical offsets and the known height of the target. 
     */
    public Pose3d TagPoseRobotSpace = new Pose3d();
}
