package frc.robot.subsystems.vision.photon;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldTargets;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionMap;

public class PhotonVisionCamera {
    private String _photonName;
    private PhotonCamera _cam;
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    private PhotonPoseEstimator _photonEstimator;

    public PhotonCameraSim Sim = null;

    public PhotonVisionCamera(String photonName, Transform3d robotCameraTransform) {
        _photonName = photonName;
        _cam = new PhotonCamera(_photonName);
        _photonEstimator = new PhotonPoseEstimator(FieldTargets.FieldTagLayout, robotCameraTransform);

        if (Robot.isSimulation()) {
            var camProps = new SimCameraProperties();
            // A 640 x 480 camera with a 100 degree diagonal FOV.
            camProps.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            camProps.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            camProps.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            camProps.setAvgLatencyMs(35);
            camProps.setLatencyStdDevMs(5);

            // The simulation of this camera. Its values used in real robot code will be updated.
            Sim = new PhotonCameraSim(_cam, camProps);
            // Enable the raw and processed streams. These are enabled by default.
            Sim.enableRawStream(true);
            Sim.enableProcessedStream(true);

            // Enable drawing a wireframe visualization of the field to the camera streams.
            // This is extremely resource-intensive and is disabled by default.
            Sim.enableDrawWireframe(false);
        }
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
            if (inputs.LatestResult == null || result.getTimestampSeconds() > inputs.LatestResult.getTimestampSeconds())
                inputs.LatestResult = result;
        }

        inputs.TimestampSeconds = inputs.LatestResult.getTimestampSeconds();

        if (inputs.LatestResult.hasTargets()) {
            inputs.TargetCount = inputs.LatestResult.getTargets().size();
            var target = inputs.LatestResult.getBestTarget();
            inputs.PrimaryTargetId = target.getFiducialId();
            inputs.PrimaryTargetRotation2d = new Rotation3d(0, target.getPitch(), target.getYaw());

            visionEst = _photonEstimator.estimateCoprocMultiTagPose(inputs.LatestResult);
            if (visionEst.isEmpty()) {
                visionEst = _photonEstimator.estimateLowestAmbiguityPose(inputs.LatestResult);
            }
            updateEstimationStdDevs(visionEst, inputs.LatestResult.getTargets(), inputs);

            visionEst.ifPresentOrElse(e -> {
                inputs.BotPoseEstimate = e.estimatedPose.toPose2d();
            }, () -> {
                inputs.BotPoseEstimate = null;
            });
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonCameraInputsAutoLogged inputs) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            inputs.CurrentStdDevs = VisionMap.kSingleTagStdDevs;
        } else {
            // Pose present. Start running heuristic
            var estStdDevs = VisionMap.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = _photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                inputs.CurrentStdDevs = VisionMap.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;

                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionMap.kMultiTagStdDevs;

                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) {
                    estStdDevs = new double[] {
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE
                    };
                } else {
                    // Vector multiplication of std devs by a distance multiplier that grows quadratically. 
                    // The 30 in the denominator is a tunable constant that determines how quickly the std devs grow with distance.
                    var distStdDevMultiplier = 1 + (avgDist * avgDist / 30);
                    estStdDevs = new double[] {
                            estStdDevs[0] * distStdDevMultiplier,
                            estStdDevs[1] * distStdDevMultiplier,
                            estStdDevs[2] * distStdDevMultiplier
                    };
                }

                // If the estimation strategy was "lowest ambiguity", increase std devs to reflect the higher uncertainty of this method.
                inputs.CurrentStdDevs = estStdDevs;
            }
        }
    }

    public void setPipeline(int pipeline) {
        _cam.setPipelineIndex(pipeline);
    }

    public void setRobotCameraTransform(Transform3d robotCameraTransform) {
        _photonEstimator.setRobotToCameraTransform(robotCameraTransform);
    }
}
