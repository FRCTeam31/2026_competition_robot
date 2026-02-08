package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.climb.ClimbInputsAutoLogged;
import frc.robot.subsystems.hopper.HopperInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveSubsystemInputsAutoLogged;
import frc.robot.subsystems.turret.TurretInputsAutoLogged;
import frc.robot.subsystems.vision.LimelightCameraInputsAutoLogged;
import frc.robot.subsystems.vision.PhotonCameraInputsAutoLogged;

public class SuperStructure {
    public static SwerveSubsystemInputsAutoLogged Swerve = new SwerveSubsystemInputsAutoLogged();

    public static HopperInputsAutoLogged Hopper = new HopperInputsAutoLogged();

    public static TurretInputsAutoLogged Turret = new TurretInputsAutoLogged();

    public static ClimbInputsAutoLogged Climb = new ClimbInputsAutoLogged();

    public static Map<String, LimelightCameraInputsAutoLogged> VisionLimelights = new HashMap<>();
    public static Map<String, PhotonCameraInputsAutoLogged> VisionPhotons = new HashMap<>();
}
