// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import org.prime.dashboard.DashboardSection;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.oi.OperatorInterface;
import frc.robot.pneumatics.Pneumatics;
import frc.robot.subsystems.leds.PwmLEDs;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.Climb.FrictionBrakeState;
import frc.robot.subsystems.climb.Climb.SupportState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;
import frc.robot.subsystems.hopper.Hopper.TransferFeedState;
import frc.robot.subsystems.climb.Climb.ClimbControlState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.UptakeState;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightVision;
import frc.robot.subsystems.vision.photon.PhotonVision;
import frc.robot.subsystems.turret.Turret.FiringState;
import frc.robot.subsystems.turret.Turret.OperatingMode;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static OperatorInterface OperatorInterface;
  public static SendableChooser<Command> AutoChooser;

  public static PwmLEDs LEDs;
  public static Swerve Swerve;
  public static LimelightVision LimelightVision;
  public static PhotonVision PhotonVision;
  public static Pneumatics Pneumatics;
  public static Hopper Hopper;
  public static Climb Climb;
  public static Turret Turret;

  public enum IntakeCombinedState {
    INWARDS,
    OUTWARDS
  }

  public static void initialize() {
    try {
      // Create dashboard sections
      AutoDashboardSection = new DashboardSection("Auto");
      // TeleopDashboardSection = new TeleopDashboardTab();
      // CommandsDashboardSection = new DashboardSection("Commands");
      // TestDashboardSection = new DashboardSection("Test");

      // Create subsystems
      // LEDs = new PwmLEDs();

      // LimelightVision = new LimelightVision();
      // LimelightVision.addCamera(VisionMap.LimelightTurretName, VisionMap.LimelightTurretTransform);
      // PhotonVision = new PhotonVision();
      // PhotonVision.addCamera(VisionMap.PhotonCam1Name, VisionMap.PhotonCam1Transform);
      // PhotonVision.addCamera(VisionMap.PhotonCam2Name, VisionMap.PhotonCam2Transform);

      // Swerve = new Swerve();
      Pneumatics = new Pneumatics();
      Hopper = new Hopper();
      Climb = new Climb();
      Turret = new Turret();

      // Create and bind the operator interface
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls();
      OperatorInterface.bindOperatorControls();

      // Register the named commands from each subsystem that may be used in PathPlanner
      // NamedCommands.registerCommands(Swerve.getNamedCommands());

      // Build an auto chooser. This will use Commands.none() as the default option.
      AutoChooser = AutoBuilder.buildAutoChooser();
      AutoDashboardSection.putData("Auto Chooser", AutoChooser);
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }

  //#region Commands

  /**
   * Starts the turret firing sequence. In AUTO mode this triggers target-seeking;
   * in MANUAL mode this immediately feeds.
   * @return Command
   */
  public static Command startShooting() {
    return Turret.setFiring(FiringState.FIRING)
        .andThen(Turret.setFeed(UptakeState.FORWARDS));
  }

  /**
   * Stops the turret firing and returns to idle
   * @return Command
   */
  public static Command stopShooting() {
    return Turret.setFeed(UptakeState.STOPPED)
        .andThen(Turret.setFiring(FiringState.IDLE));
  }

  /**
   * Command to set the hopper and intake into the correct positions
   * when starting a match
   * @return Command
   */
  public static Command homeRobotCommand() {
    return Hopper.setFeed(TransferFeedState.INWARDS)
        .andThen(Hopper.setHopperIntakeControl(HopperIntakeState.OUT))
        // Time for intake to fully extend
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setIntakeFeed(IntakeFeedState.INWARDS));
  }

  public static Command setIntakeStates(IntakeCombinedState state) {
    if (state == IntakeCombinedState.INWARDS) {
      return Hopper.setIntakeFeed(IntakeFeedState.INWARDS)
          .andThen(Hopper.setFeed(TransferFeedState.INWARDS))
          .andThen(Turret.setFeed(UptakeState.FORWARDS))
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    } else {
      return Hopper.setIntakeFeed(IntakeFeedState.OUTWARDS)
          .andThen(Hopper.setFeed(TransferFeedState.OUTWARDS))
          .andThen(Turret.setFeed(UptakeState.REVERSED))
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

  }

  public static Command setupClimb() {
    return Commands.runOnce(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.SETUP_IN_PROGRESS)
        .andThen(setIntakeStates(IntakeCombinedState.OUTWARDS))
        .andThen(Climb.setBrake(FrictionBrakeState.RELEASED))
        .andThen(Climb.setClimb(ClimbState.UP))
        // Time to dump all fuel
        .andThen(Commands.waitSeconds(1))
        .andThen(Hopper.setFeed(TransferFeedState.STOPPED))
        .andThen(Hopper.setIntakeFeed(IntakeFeedState.STOPPED))
        .andThen(Hopper.setHopperIntakeControl(HopperIntakeState.IN))
        // Time for intake to fully raise
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setSupport(SupportState.LOWERED))
        .andThen(Commands.runOnce(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.SETUP_DONE))
        .andThen(OperatorInterface.rumbleControllerShort(OperatorInterface.DriverController))
        .onlyIf(() -> SuperStructure.Climb.ClimbControlState == ClimbControlState.RESET)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Command to run when the robot should start climbing
   * 
   * @return Command
   */
  public static Command startClimbing() {
    return Commands.runOnce(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.CLIMBING_UP)
        // Wait until climb is fully lowered
        .andThen(Commands.waitUntil(() -> SuperStructure.Climb.LowerLimitSwitch))
        .andThen(Climb.setBrake(FrictionBrakeState.APPLIED))
        .andThen(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.HAS_CLIMBED)
        .onlyIf(() -> SuperStructure.Climb.ClimbControlState == ClimbControlState.SETUP_DONE)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Command to run when decending after a climb
   * 
   * @return Command
   */
  public static Command stopClimbing() {
    return Commands.runOnce(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.CLIMBING_DOWN)
        .andThen(Climb.setBrake(FrictionBrakeState.RELEASED))
        // Time for brake to fully release
        .andThen(Commands.waitSeconds(1))
        .andThen(Climb.setClimb(ClimbState.UP))
        .andThen(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.CLIMBING_DONE)
        .onlyIf(() -> SuperStructure.Climb.ClimbControlState == ClimbControlState.HAS_CLIMBED)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Resets the robot to the correct state following a climb
   * 
   * @return Command
   */
  public static Command resetRobotAfterClimb() {
    return Commands.runOnce(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.RESETTING)
        .andThen(Climb.setSupport(SupportState.RAISED))
        // Time for support to fully raise
        .andThen(Commands.waitSeconds(1))
        .andThen(homeRobotCommand())
        .andThen(() -> SuperStructure.Climb.ClimbControlState = ClimbControlState.RESET)
        .onlyIf(() -> SuperStructure.Climb.ClimbControlState == ClimbControlState.CLIMBING_DONE ||
            SuperStructure.Climb.ClimbControlState == ClimbControlState.SETUP_DONE)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }
  //#endregion

  public static Command toggleShooterOn() {
    return Turret.setFiring(FiringState.FIRING)
        .andThen(Commands.waitUntil(() -> SuperStructure.Turret.FlywheelAtTargetSpeed &&
            SuperStructure.Turret.YawOnTarget &&
            SuperStructure.Turret.HoodOnTarget))
        .andThen(Turret.setFeed(UptakeState.FORWARDS))
        .andThen(() -> SuperStructure.Hopper.TransferFeedState = TransferFeedState.INWARDS);
  }

  public static Command toggleShooterOff() {
    return Turret.setFiring(FiringState.IDLE)
        .andThen(Turret.setFeed(UptakeState.STOPPED))
        .andThen(() -> SuperStructure.Hopper.TransferFeedState = TransferFeedState.STOPPED);
  }

  public static Command startAuto() {
    return Turret.setOperatingMode(OperatingMode.AUTO)
        .andThen(Turret.setFiring(FiringState.IDLE));
  }

  public static Command toggleIntakeOn() {
    return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.OUT)
        .andThen(() -> SuperStructure.Hopper.IntakeFeedState = IntakeFeedState.INWARDS);
  }

  public static Command toggleIntakeOff() {
    return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.IN)
        .andThen(() -> SuperStructure.Hopper.IntakeFeedState = IntakeFeedState.STOPPED);
  }

  public static Command toggleClimbArmOn() {
    return Commands.runOnce(() -> SuperStructure.Climb.SupportState = SupportState.RAISED);
  }

  // public static Command toggleIntakeOff() {
  //   return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.IN)
  //       .andThen(() -> SuperStructure.Turret.FeedState = UptakeState.STOPPED);
  // }

  public static Map<String, Supplier<Command>> getNamedCommandSuppliers() {
    return Map.of(
        "Enable_Autonomous_Shooting", () -> toggleShooterOn(),
        "Disable_Autonomous_Shooting", () -> toggleShooterOff(),
        "Start_Auto", () -> startAuto(),
        "Take_Out_And_Enable_Intake", () -> toggleIntakeOn(),
        "Put_In_And_Disable_Intake", () -> toggleIntakeOff(),
        "ClimbSequence", () -> setupClimb().andThen(startClimbing()).andThen(stopClimbing()));
  }

}