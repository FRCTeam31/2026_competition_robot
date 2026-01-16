package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.swerve.SwerveSubsystemInputsAutoLogged;
import frc.robot.subsystems.vision.LimelightInputsAutoLogged;
import frc.robot.subsystems.vision.LimelightNameEnum;

public class SuperStructure {
    public static SwerveSubsystemInputsAutoLogged Swerve = new SwerveSubsystemInputsAutoLogged();
    public static HopperInputsAutoLogged Hopper = new HopperInputsAutoLogged();
    public static Map<LimelightNameEnum, LimelightInputsAutoLogged> Limelights = new HashMap<>();
}
