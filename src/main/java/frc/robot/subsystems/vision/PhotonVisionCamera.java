package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionCamera {
    private String _photonName;
    private PhotonCamera _cam;
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    private PhotonPoseEstimator _photonEstimator;
    private static final AprilTagFieldLayout _tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public PhotonVisionCamera(String photonName) {
        _photonName = photonName;
        _cam = new PhotonCamera(_photonName);
        _photonEstimator = new PhotonPoseEstimator(_tagLayout, new Transform3d());
    }

    public void updateInputs(PhotonCameraInputsAutoLogged inputs) {
        inputs.LatestResult = _cam.getLatestResult();

        if (inputs.LatestResult.hasTargets()) {
            var target = inputs.LatestResult.getBestTarget();
            inputs.TargetPitch = target.getPitch();
            inputs.TargetYaw = target.getYaw();

            visionEst = _photonEstimator.estimateCoprocMultiTagPose(inputs.LatestResult);
            if (visionEst.isEmpty()) {
                visionEst = _photonEstimator.estimateLowestAmbiguityPose(inputs.LatestResult);
            }

            visionEst.ifPresent(e -> {
                inputs.BotPoseEstimate = e.estimatedPose.toPose2d();
            });

            inputs.TimestampSeconds = inputs.LatestResult.getTimestampSeconds();
        }
    }

    public void setPipeline(int pipeline) {
        _cam.setPipelineIndex(pipeline);
    }

    public void setRobotCameraTransform(Transform3d robotCameraTransform) {
        _photonEstimator.setRobotToCameraTransform(robotCameraTransform);
    }
}
