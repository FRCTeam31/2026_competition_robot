package frc.robot.subsystems.vision.photon;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

@AutoLog
public class PhotonCameraInputs {

    public PhotonPipelineResult LatestResult = new PhotonPipelineResult();

    public int TargetCount = 0;
    public int PrimaryTargetId = -1;
    public Rotation3d PrimaryTargetRotation2d = new Rotation3d();

    /**
     * The robot's estiamted pose in field space.
     */
    public Pose2d BotPoseEstimate = new Pose2d();

    public double TimestampSeconds = 0.0;
}
