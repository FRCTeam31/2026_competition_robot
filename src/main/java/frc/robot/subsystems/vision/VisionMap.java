package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionMap {
        public static final String LimelightTurretName = "limelight-turret";
        public static final Transform3d LimelightTurretTransform = new Transform3d();

        public static final String PhotonCam1Name = "leftCam";
        public static final Transform3d PhotonCam1Transform = new Transform3d(
                        new Translation3d(
                                        // -0.1707855,
                                        -0.0331175,
                                        0.298581,
                                        0.4786495 + 0.0254),
                        new Rotation3d(0, 0, Math.PI / 2));
        public static final String PhotonCam2Name = "rightCam";
        public static final Transform3d PhotonCam2Transform = new Transform3d(
                        new Translation3d(
                                        -0.0331175,
                                        -0.298581,
                                        0.4786495 + 0.0254),
                        new Rotation3d(0, 0, 3 * Math.PI / 2));
        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final double[] kSingleTagStdDevs = { 4, 4, 8 };
        public static final double[] kMultiTagStdDevs = { 0.5, 0.5, 1 };
        public static final double PHOTON_MAX_AVG_TAG_DISTANCE_METERS = 3;
}
