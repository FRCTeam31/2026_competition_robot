package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;

public interface IVisionSubsystem {
    boolean addCamera(String name, Transform3d robotCameraTransform);

    boolean removeCamera(String name);

    java.util.Set<String> getCameraNames();

    boolean hasCamera(String name);

    void setPipeline(String name, int pipeline);

    void setCameraPose(String name, Pose3d pose);

    void periodic();

    Command setProcessingPipeline(String name, int pipeline);
}
