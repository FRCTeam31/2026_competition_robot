package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionMap {
    public static final String LimelightTurretName = "limelight-turret";
    public static final Transform3d LimelightTurretTransform = new Transform3d();

    public static final String PhotonCam1Name = "photon-cam-1";
    public static final Transform3d PhotonCam1Transform = new Transform3d();
    public static final String PhotonCam2Name = "photon-cam-2";
    public static final Transform3d PhotonCam2Transform = new Transform3d();
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final double[] kSingleTagStdDevs = { 4, 4, 8 };
    public static final double[] kMultiTagStdDevs = { 0.5, 0.5, 1 };
}
