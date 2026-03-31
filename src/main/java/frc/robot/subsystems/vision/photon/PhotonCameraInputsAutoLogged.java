package frc.robot.subsystems.vision.photon;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionMap;

// No @AutoLog — we implement LoggableInputs manually
public class PhotonCameraInputsAutoLogged implements LoggableInputs {

    public PhotonPipelineResult LatestResult = new PhotonPipelineResult();
    public int TargetCount = 0;
    public int PrimaryTargetId = -1;
    public Rotation3d PrimaryTargetRotation2d = new Rotation3d();
    public Pose2d BotPoseEstimate = new Pose2d();
    public double[] CurrentStdDevs = VisionMap.kSingleTagStdDevs;
    public double TimestampSeconds = 0.0;

    @Override
    public void toLog(LogTable table) {
        table.put("LatestResult", PhotonPipelineResult.proto, LatestResult);
        table.put("TargetCount", TargetCount);
        table.put("PrimaryTargetId", PrimaryTargetId);
        table.put("PrimaryTargetRotation2d", PrimaryTargetRotation2d);
        table.put("BotPoseEstimate", BotPoseEstimate);
        table.put("CurrentStdDevs", CurrentStdDevs);
        table.put("TimestampSeconds", TimestampSeconds);
    }

    @Override
    public void fromLog(LogTable table) {
        LatestResult = table.get("LatestResult", PhotonPipelineResult.proto, new PhotonPipelineResult());
        TargetCount = (int) table.get("TargetCount", TargetCount);
        PrimaryTargetId = (int) table.get("PrimaryTargetId", PrimaryTargetId);
        PrimaryTargetRotation2d = table.get("PrimaryTargetRotation2d", PrimaryTargetRotation2d);
        BotPoseEstimate = table.get("BotPoseEstimate", Pose2d.struct, BotPoseEstimate);
        CurrentStdDevs = table.get("CurrentStdDevs", CurrentStdDevs);
        TimestampSeconds = table.get("TimestampSeconds", TimestampSeconds);
    }
}