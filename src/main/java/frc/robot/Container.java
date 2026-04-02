// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.dashboard.DriverDashboard;
import frc.robot.oi.OperatorInterface;
import frc.robot.pneumatics.Pneumatics;
import frc.robot.subsystems.leds.PwmLEDs;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;
import frc.robot.subsystems.hopper.Hopper.TransferFeedState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightVision;
import frc.robot.subsystems.vision.photon.PhotonVision;

public class Container {
  public static DriverDashboard Dashboard;
  public static OperatorInterface OperatorInterface;
  public static SendableChooser<Command> AutoChooser;

  public static PwmLEDs LEDs;
  public static Swerve Swerve;
  public static LimelightVision LimelightVision;
  public static PhotonVision PhotonVision;
  public static Pneumatics Pneumatics;
  public static Hopper Hopper;
  public static Turret Turret;
  public static PowerDistribution PD;

  public enum IntakeCombinedState {
    INWARDS,
    OUTWARDS
  }

  public static void initialize() {
    try {
      OperatorInterface = new OperatorInterface();
      // Create dashboard
      Dashboard = new DriverDashboard();

      // Create subsystems
      LEDs = new PwmLEDs();

      LimelightVision = new LimelightVision();
      LimelightVision.addCamera(VisionMap.LimelightTurretName, VisionMap.LimelightTurretTransform);
      PhotonVision = new PhotonVision();
      PhotonVision.addCamera(VisionMap.PhotonCam1Name, VisionMap.PhotonCam1Transform);
      PhotonVision.addCamera(VisionMap.PhotonCam2Name, VisionMap.PhotonCam2Transform);

      Swerve = new Swerve();
      Pneumatics = new Pneumatics();
      Hopper = new Hopper();
      Turret = new Turret();
      PD = new PowerDistribution(1, ModuleType.kRev);

      // Create and bind the operator interface
      OperatorInterface.bindDriverControls();
      OperatorInterface.bindOperatorControls();

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(getNamedCommandSuppliers());

      // Build an auto chooser. This will use Commands.none() as the default option.
      AutoChooser = AutoBuilder.buildAutoChooser();
      Dashboard.putData("Auto Chooser", AutoChooser);
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
  // public static Command startShooting() {
  //   return Turret.setFiring(FiringState.FIRING)
  //       .andThen(Hopper.setFeed(TransferFeedState.INWARDS))
  //       .andThen(Turret.setFeed(UptakeState.FORWARDS));
  // }

  // public static Command startShooting(DoubleSupplier speed) {
  //   return Turret.setShooterCommand(speed.getAsDouble())
  //       .andThen(Hopper.setFeed(TransferFeedState.INWARDS))
  //       .andThen(Turret.setFeedCommand(0.4)).repeatedly();
  // }

  /**
   * Stops the turret firing and returns to idle
   * @return Command
   */
  // public static Command stopShooting() {
  //   return Turret.setFiring(FiringState.IDLE)
  //       .andThen(Hopper.setFeed(TransferFeedState.STOPPED))
  //       .andThen(Turret.setFeed(UptakeState.STOPPED));
  // }

  // public static Command stopShooting() {
  //   return Turret.stopShooterCommand()
  //       .andThen(Hopper.setFeed(TransferFeedState.STOPPED))
  //       .andThen(Turret.stopFeedCommand());
  // }

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
          // .andThen(Turret.setFeed(UptakeState.FORWARDS))
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    } else {
      return Hopper.setIntakeFeed(IntakeFeedState.OUTWARDS)
          .andThen(Hopper.setFeed(TransferFeedState.OUTWARDS))
          // .andThen(Turret.setFeed(UptakeState.REVERSED))
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

  }
  //#endregion

  // public static Command toggleShooterOn() {
  //   return Turret.setFiring(FiringState.FIRING)
  //       .andThen(Commands.waitUntil(() -> SuperStructure.Turret.FlywheelAtTargetSpeed &&
  //           SuperStructure.Turret.YawOnTarget &&
  //           SuperStructure.Turret.HoodOnTarget))
  //       .andThen(Turret.setFeed(UptakeState.FORWARDS))
  //       .andThen(() -> SuperStructure.Hopper.TransferFeedState = TransferFeedState.INWARDS);
  // }

  // public static Command toggleShooterOff() {
  //   return Turret.setFiring(FiringState.IDLE)
  //       .andThen(Turret.setFeed(UptakeState.STOPPED))
  //       .andThen(() -> SuperStructure.Hopper.TransferFeedState = TransferFeedState.STOPPED);
  // }

  // public static Command startAuto() {
  //   return Turret.setOperatingMode(OperatingMode.AUTO)
  //       .andThen(Turret.setFiring(FiringState.IDLE));
  // }

  public static Command toggleIntakeOn() {
    return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.OUT)
        .andThen(() -> SuperStructure.Hopper.IntakeFeedState = IntakeFeedState.INWARDS)
        .andThen(() -> SuperStructure.Hopper.TransferFeedState = TransferFeedState.STOPPED); //These have been changed for our current strategy for defense
  }

  public static Command toggleIntakeOff() {
    return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.IN)
        .andThen(() -> SuperStructure.Hopper.IntakeFeedState = IntakeFeedState.STOPPED);
  }

  public static Command ejectBalls() {
    return Commands.runOnce(() -> SuperStructure.Hopper.IntakeFeedState = IntakeFeedState.OUTWARDS)
        .andThen(() -> SuperStructure.Hopper.TransferFeedState = TransferFeedState.OUTWARDS);
  }

  // public static Command toggleIntakeOff() {
  //   return Commands.runOnce(() -> SuperStructure.Hopper.IntakeControlState = HopperIntakeState.IN)
  //       .andThen(() -> SuperStructure.Turret.FeedState = UptakeState.STOPPED);
  // }

  public static List<Pair<String, Command>> getNamedCommandSuppliers() {
    return List.of(
        // Pair.of("Disable_Autonomous_Shooting", toggleShooterOff()),
        // Pair.of("Start_Auto", startAuto()),
        Pair.of("Take_Out_And_Enable_Intake", toggleIntakeOn()),
        Pair.of("Put_In_And_Disable_Intake", toggleIntakeOff()),
        Pair.of("Dump_Balls", ejectBalls()));
  }
}