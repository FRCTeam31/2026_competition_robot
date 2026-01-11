// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.routine.BuildableAutoRoutine;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static BuildableAutoRoutine AutoBuilder;

  public static PwmLEDs LEDs;
  public static Swerve Swerve;
  public static Vision Vision;
  public static OperatorInterface OperatorInterface;

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

      // Create and bind the operator interface
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(Swerve, Vision);
      OperatorInterface.bindOperatorControls(Swerve, Vision);

      // Create our custom auto builder
      AutoBuilder = new BuildableAutoRoutine(getNamedCommandSuppliers());
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }

  //#region Commands

  // Commands that utilize multiple subsystems will go here

  public static Map<String, Supplier<Command>> getNamedCommandSuppliers() {
    return Map.of();
  }
  //#endregion
}
