package frc.robot.subsystems.vision.photon;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionCamera {
    private String _photonName;
    private PhotonCamera _cam;
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    private PhotonPoseEstimator _photonEstimator;
    private static final AprilTagFieldLayout _tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public PhotonVisionCamera(String photonName, Transform3d robotCameraTransform) {
        _photonName = photonName;
        _cam = new PhotonCamera(_photonName);
        _photonEstimator = new PhotonPoseEstimator(_tagLayout, robotCameraTransform);
    }

    public void updateInputs(PhotonCameraInputsAutoLogged inputs) {
        var results = _cam.getAllUnreadResults();
        if (results.isEmpty()) {
            inputs.LatestResult = null;
            inputs.TargetCount = 0;
            inputs.PrimaryTargetId = -1;
            inputs.PrimaryTargetRotation2d = new Rotation3d();
            inputs.BotPoseEstimate = null;
            inputs.TimestampSeconds = -1;
            return;
        }

        // Take the latest result from the list
        for (var result : results) {
            if (result.getTimestampSeconds() > inputs.LatestResult.getTimestampSeconds())
                inputs.LatestResult = result;
        }

        if (inputs.LatestResult.hasTargets()) {
            inputs.TargetCount = inputs.LatestResult.getTargets().size();
            var target = inputs.LatestResult.getBestTarget();
            inputs.PrimaryTargetId = target.getFiducialId();
            inputs.PrimaryTargetRotation2d = new Rotation3d(0, target.getPitch(), target.getYaw());

            visionEst = _photonEstimator.estimateCoprocMultiTagPose(inputs.LatestResult);
            if (visionEst.isEmpty()) {
                visionEst = _photonEstimator.estimateLowestAmbiguityPose(inputs.LatestResult);
            }

            visionEst.ifPresentOrElse(e -> {
                inputs.BotPoseEstimate = e.estimatedPose.toPose2d();
            }, () -> {
                inputs.BotPoseEstimate = null;
            });
        }
    }

    public void setPipeline(int pipeline) {
        _cam.setPipelineIndex(pipeline);
    }

    public void setRobotCameraTransform(Transform3d robotCameraTransform) {
        _photonEstimator.setRobotToCameraTransform(robotCameraTransform);
    }
}
