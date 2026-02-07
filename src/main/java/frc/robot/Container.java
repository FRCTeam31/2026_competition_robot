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
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb.FrictionBrakeState;
import frc.robot.subsystems.climb.Climb.SupportState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.ExtensionState;
import frc.robot.subsystems.hopper.Hopper.IntakeControlState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;
import frc.robot.subsystems.hopper.Hopper.TransferFeedState;
import frc.robot.subsystems.climb.Climb.ClimbControlState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.UptakeState;
import frc.robot.subsystems.turret.Turret.FlywheelState;
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
  public static Turret _turret;

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
      _turret = new Turret(isReal);

      // Create and bind the operator interface
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(Swerve, Vision, _turret, Climb, Hopper);
      OperatorInterface.bindOperatorControls(Swerve, Vision, _turret, Climb, Hopper);

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
  public static Command startShooting() {
    return _turret.setFlywheel(FlywheelState.SHOOTING)
        .andThen(_turret.setFeed(UptakeState.FORWARDS));
  }

  /**
   * Stop the turret flywheel and feed
   * @return Command
   */
  public static Command stopShooting() {
    return _turret.setFeed(UptakeState.STOPPED)
        .andThen(_turret.setFlywheel(FlywheelState.IDLE));
  }

  /**
   * Command to set the hopper and intake into the correct positions
   * when starting a match
   * @return Command
   */
  public static Command homeRobotCommand() {
    return Hopper.setFeed(TransferFeedState.INWARDS)
        .andThen(Hopper.setHopper(ExtensionState.OUT))
        // Time for intake to fully extend
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setIntakeControl(IntakeControlState.OUT))
        .andThen(Hopper.setIntakeFeed(IntakeFeedState.INWARDS));
  }

  public static Command setupClimb() {
    return Hopper.setIntakeFeed(IntakeFeedState.OUTWARDS)
        .andThen(Hopper.setFeed(TransferFeedState.OUTWARDS))
        .andThen(_turret.setFeed(UptakeState.REVERSED))
        .andThen(Climb.setBrake(FrictionBrakeState.RELEASED))
        .andThen(Climb.setClimb(ClimbState.UP))
        // Time to dump all fuel
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setFeed(TransferFeedState.STOPPED))
        .andThen(Hopper.setIntakeFeed(IntakeFeedState.STOPPED))
        .andThen(Hopper.setIntakeControl(IntakeControlState.IN))
        // Time for intake to fully raise
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setHopper(ExtensionState.IN))
        // Time for hopper to fully reverse extension
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setSupport(SupportState.LOWERED))
        .andThen(Commands.runOnce(() -> SuperStructure.Climb.climbControlState = ClimbControlState.SETUP_DONE))
        .onlyIf(() -> SuperStructure.Climb.climbControlState == ClimbControlState.RESET)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Command to run when the robot should start climbing
   * 
   * @return Command
   */
  public static Command startClimbing() {
    return Commands.runOnce(() -> SuperStructure.Climb.climbControlState = ClimbControlState.CLIMBING_UP)
        // Time for climb to fully lower
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setBrake(FrictionBrakeState.APPLIED))
        .andThen(() -> SuperStructure.Climb.climbControlState = ClimbControlState.HAS_CLIMBED)
        .onlyIf(() -> SuperStructure.Climb.climbControlState == ClimbControlState.SETUP_DONE)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Command to run when decending after a climb
   * 
   * @return Command
   */
  public static Command stopClimbing() {
    return Commands.runOnce(() -> SuperStructure.Climb.climbControlState = ClimbControlState.CLIMBING_DOWN)
        .andThen(Climb.setBrake(FrictionBrakeState.RELEASED))
        // Time for brake to fully release
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setClimb(ClimbState.UP))
        .andThen(() -> SuperStructure.Climb.climbControlState = ClimbControlState.CLIMBING_DONE)
        .onlyIf(() -> SuperStructure.Climb.climbControlState == ClimbControlState.HAS_CLIMBED)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Resets the robot to the correct state following a climb
   * 
   * @return Command
   */
  public static Command resetRobotAfterClimb() {
    return Commands.runOnce(() -> SuperStructure.Climb.climbControlState = ClimbControlState.RESETTING)
        .andThen(Climb.setSupport(SupportState.RAISED))
        // Time for support to fully raise
        .andThen(Commands.waitSeconds(1))
        .andThen(homeRobotCommand())
        .andThen(() -> SuperStructure.Climb.climbControlState = ClimbControlState.RESET)
        .onlyIf(() -> SuperStructure.Climb.climbControlState == ClimbControlState.CLIMBING_DONE ||
            SuperStructure.Climb.climbControlState == ClimbControlState.SETUP_DONE)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }
  //#endregion
}
