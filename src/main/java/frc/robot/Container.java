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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.pneumatics.Pneumatics;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
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
  public static Turret Turret;

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
      Turret = new Turret(isReal);

      // Create and bind the operator interface
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(Swerve, Vision, Turret, Climb, Hopper);
      OperatorInterface.bindOperatorControls(Swerve, Vision, Turret, Climb, Hopper);

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

  /**
   * Enables the turret flywheel and sets its feed inwards
   * @return Command
   */
  public Command startShooting() {
    return Turret.setFlywheelShooting()
        .andThen(Turret.setFeedForward());
  }

  /**
   * Stop the turret flywheel and feed
   * @return Command
   */
  public Command stopShooting() {
    return Turret.stopFeed()
        .andThen(Turret.setFlywheelIdle());
  }

  /**
   * Command to set the hopper and intake into the correct positions
   * when starting a match
   * @return Command
   */
  public Command robotStartingCommand() {
    return Hopper.setFeedInwards()
        .andThen(Hopper.setHopperOut())
        // Time for intake to fully extend
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setIntakeOut())
        .andThen(Hopper.setIntakeFeedInwards());
  }

  public Command setupClimb() {
    return Hopper.setIntakeFeedOutwards()
        .andThen(Hopper.setFeedOutwards())
        .andThen(Turret.setFeedReverse())
        .andThen(Climb.setBrakeReleased())
        .andThen(Climb.setClimbUp())
        // Time to dump all fuel
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.stopFeed())
        .andThen(Hopper.stopIntakeFeed())
        .andThen(Hopper.setIntakeIn())
        // Time for intake to fully raise
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setHopperIn())
        // Time for hopper to fully reverse extension
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setSupportLowered());
  }

  public Command startClimbing() {
    return Climb.setClimbDown()
        // Time for climb to fully lower
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setBrakeApplied());
  }

  public Command stopClimbing() {
    return Climb.setBrakeReleased()
        // Time for brake to fully release
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setClimbUp());
  }

  public Command endClimb() {
    return Climb.setSupportRaised()
        // Time for support to fully raise
        .andThen(Commands.waitSeconds(1))
        .andThen(robotStartingCommand());
  }
  //#endregion
}
