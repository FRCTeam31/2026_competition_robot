package frc.robot.subsystems.vision.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructure;
import frc.robot.subsystems.vision.IVisionSubsystem;
import frc.robot.subsystems.vision.VisionMap;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.prime.subsystems.LoggedSubsystem;

public class LimelightVision extends LoggedSubsystem implements IVisionSubsystem {
    private Map<String, LimeLightCamera> _limelights = new HashMap<>();

    public LimelightVision() {
        setName("LimelightVision");

        // Add default turret limelight
        addCamera(VisionMap.LimelightTurretName, VisionMap.LimelightTurretTransform);
    }

    /**
     * Adds a new limelight to the vision subsystem.
     * @param name The network table name of the limelight
     * @return true if the limelight was added, false if it already exists
     */
    public boolean addCamera(String name, Transform3d robotCameraTransform) {
        if (_limelights.containsKey(name)) {
            return false;
        }

        _limelights.put(name, new LimeLightCamera(name));
        SuperStructure.VisionLimelights.put(name, new LimelightCameraInputsAutoLogged());
        var robotCameraPose = new Pose3d(robotCameraTransform.getTranslation(), robotCameraTransform.getRotation());
        setCameraPose(name, robotCameraPose);
        return true;
    }

    /**
     * Removes a limelight from the vision subsystem.
     * @param name The network table name of the limelight to remove
     * @return true if the limelight was removed, false if it didn't exist
     */
    public boolean removeCamera(String name) {
        if (!_limelights.containsKey(name)) {
            return false;
        }

        LimeLightCamera ll = _limelights.remove(name);
        if (ll != null) {
            ll.close();
        }
        SuperStructure.VisionLimelights.remove(name);
        return true;
    }

    /**
     * Gets the set of all limelight names currently managed by this subsystem.
     * @return Set of limelight names
     */
    public Set<String> getCameraNames() {
        return _limelights.keySet();
    }

    /**
     * Checks if a limelight with the given name exists.
     * @param name The network table name of the limelight
     * @return true if the limelight exists
     */
    public boolean hasCamera(String name) {
        return _limelights.containsKey(name);
    }

    /**
     * Sets limelight's LED state.
     *    0 = use the LED Mode set in the current pipeline.
     *    1 = force off.
     *    2 = force blink.
     *    3 = force on.
     * @param name The name of the desired limelight
     * @param mode The LED mode to set
     */
    public void setLEDMode(String name, int mode) {
        if (_limelights.containsKey(name)) {
            _limelights.get(name).setLedMode(mode);
        }
    }

    /**
     * Forces the LED to blink a specified number of times, then returns to pipeline control.
     * @param name The name of the desired limelight
     * @param blinkCount The number of times to blink the LED
     */
    public void blinkLED(String name, int blinkCount) {
        if (_limelights.containsKey(name)) {
            _limelights.get(name).blinkLed(blinkCount);
        }
    }

    /**
     * Sets limelight's active vision pipeline.
     * @param name The name of the desired limelight
     * @param pipeline The pipeline to set active
     */
    public void setPipeline(String name, int pipeline) {
        if (_limelights.containsKey(name)) {
            _limelights.get(name).setPipeline(pipeline);
        }
    }

    /**
     * Sets limelight's streaming mode.
     *    0 = Standard - Side-by-side streams if a webcam is attached to Limelight
     *    1 = PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     *    2 = PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     * @param name The name of the desired limelight
     * @param mode The streaming mode to set
     */
    public void setPiPStreamingMode(String name, int mode) {
        if (_limelights.containsKey(name)) {
            _limelights.get(name).setPiPStreamingMode(mode);
        }
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param name The name of the desired limelight
     * @param pose The Camera's pose to set in Robot space
     */
    public void setCameraPose(String name, Pose3d pose) {
        if (_limelights.containsKey(name)) {
            _limelights.get(name).setCameraPose(pose);
        }
    }

    @Override
    public void periodic() {
        // Update superstructure
        for (var name : _limelights.keySet()) {
            _limelights.get(name).updateInputs(SuperStructure.VisionLimelights.get(name));
            // Logger.processInputs("Vision/LL/" + name, SuperStructure.LimelightStates.get(name));
        }
    }

    //#region Commands

    public Command setProcessingPipeline(String name, int pipeline) {
        return Commands.runOnce(() -> setPipeline(name, pipeline));
    }

    //#endregion
}
