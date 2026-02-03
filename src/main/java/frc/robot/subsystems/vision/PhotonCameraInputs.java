package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

@AutoLog
public class PhotonCameraInputs {

    public PhotonPipelineResult LatestResult = new PhotonPipelineResult();

    /**
     * Target pitch and yaw angles (pitch, yaw, roll format)
     */
    public double TargetPitch = 0.0;
    public double TargetYaw = 0.0;

    /**
     * The robot's estiamted pose in field space.
     */
    public Pose2d BotPoseEstimate = new Pose2d();
    public Transform3d BotTransformEstimate = new Transform3d();

    public double TimestampSeconds = 0.0;
}
