// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.pneumatics.Pneumatics;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static OperatorInterface OperatorInterface;
  public static SendableChooser<Command> AutoChooser;

  public static PwmLEDs LEDs;
  public static Swerve Swerve;
  public static Vision Vision;
  public static Pneumatics Pneumatics;
  public static Hopper Hopper;
  public static Climb Climb;

  public static void initialize(boolean isReal) {
    try {
      // Create dashboard sections
      AutoDashboardSection = new DashboardSection("Auto");
      TeleopDashboardSection = new TeleopDashboardTab();
      CommandsDashboardSection = new DashboardSection("Commands");
      TestDashboardSection = new DashboardSection("Test");

      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new Vision();
      Swerve = new Swerve(isReal);
      Pneumatics = new Pneumatics(isReal);
      Hopper = new Hopper(isReal);
      Climb = new Climb(isReal);

      // Create and bind the operator interface
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(Swerve, Vision);
      OperatorInterface.bindOperatorControls(Swerve, Vision);

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Swerve.getNamedCommands());

      // Build an auto chooser. This will use Commands.none() as the default option.
      AutoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", AutoChooser);
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }

  //#region Commands

  //#endregion
}
