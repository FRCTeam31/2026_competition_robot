package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.hopper.HopperInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveSubsystemInputsAutoLogged;
import frc.robot.subsystems.swerve.module.SwerveModuleInputsAutoLogged;
import frc.robot.subsystems.turret.TurretInputsAutoLogged;
import frc.robot.subsystems.vision.limelight.LimelightCameraInputsAutoLogged;
import frc.robot.subsystems.vision.photon.PhotonCameraInputsAutoLogged;

/**
 * This class is a container for all of the inputs from the robot's subsystems. It is used to log all of the inputs in one place, and to make it easier to access the inputs from other classes (such as the dashboard).
 */
public class SuperStructure {
    public static SwerveSubsystemInputsAutoLogged Swerve = new SwerveSubsystemInputsAutoLogged();

    public static HopperInputsAutoLogged Hopper = new HopperInputsAutoLogged();

    public static TurretInputsAutoLogged Turret = new TurretInputsAutoLogged();

    public static Map<String, LimelightCameraInputsAutoLogged> VisionLimelights = new HashMap<>();

    public static Map<String, PhotonCameraInputsAutoLogged> VisionPhotons = new HashMap<>();

    public static SwerveModuleInputsAutoLogged[] SwerveModules = new SwerveModuleInputsAutoLogged[4];
}
