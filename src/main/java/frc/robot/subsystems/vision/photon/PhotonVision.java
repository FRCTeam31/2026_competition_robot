package frc.robot.subsystems.vision.photon;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;
import frc.robot.subsystems.vision.IVisionSubsystem;

public class PhotonVision extends SubsystemBase implements IVisionSubsystem {
    public static final String DefaultCameraName = "PhotonVision";
    private Map<String, PhotonVisionCamera> _cameras = new HashMap<>();

    public PhotonVision() {
        setName("PhotonVision");

        // Add default camera(s) here
    }

    /**
     * Adds a new PhotonVision camera to the vision subsystem.
     * @param name The network table name of the camera
     * @return true if the camera was added, false if it already exists
     */
    @Override
    public boolean addCamera(String name, Transform3d robotCameraTransform) {
        if (_cameras.containsKey(name)) {
            return false;
        }

        _cameras.put(name, new PhotonVisionCamera(name, robotCameraTransform));
        SuperStructure.VisionPhotons.put(name, new PhotonCameraInputsAutoLogged());
        return true;
    }

    /**
     * Removes a PhotonVision camera from the vision subsystem.
     * @param name The network table name of the camera to remove
     * @return true if the camera was removed, false if it didn't exist
     */
    @Override
    public boolean removeCamera(String name) {
        if (!_cameras.containsKey(name)) {
            return false;
        }

        _cameras.remove(name);
        SuperStructure.VisionPhotons.remove(name);
        return true;
    }

    /**
     * Gets the set of all camera names currently managed by this subsystem.
     * @return Set of camera names
     */
    @Override
    public Set<String> getCameraNames() {
        return _cameras.keySet();
    }

    /**
     * Checks if a camera with the given name exists.
     * @param name The network table name of the camera
     * @return true if the camera exists
     */
    @Override
    public boolean hasCamera(String name) {
        return _cameras.containsKey(name);
    }

    /**
     * Sets PhotonVision camera's active vision pipeline.
     * @param name The name of the desired camera
     * @param pipeline The pipeline to set active
     */
    @Override
    public void setPipeline(String name, int pipeline) {
        if (_cameras.containsKey(name)) {
            _cameras.get(name).setPipeline(pipeline);
        }
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param name The name of the desired camera
     * @param pose The Camera's pose to set in Robot space
     */
    @Override
    public void setCameraPose(String name, Pose3d pose) {
        if (_cameras.containsKey(name)) {
            _cameras.get(name).setRobotCameraTransform(new Transform3d(pose.getTranslation(), pose.getRotation()));
        }
    }

    @Override
    public void periodic() {
        // Update superstructure
        for (var name : _cameras.keySet()) {
            _cameras.get(name).updateInputs(SuperStructure.VisionPhotons.get(name));
        }
    }

    //#region Commands

    @Override
    public Command setProcessingPipeline(String name, int pipeline) {
        return Commands.runOnce(() -> setPipeline(name, pipeline));
    }

    //#endregion
}
