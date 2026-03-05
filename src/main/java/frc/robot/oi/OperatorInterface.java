package frc.robot.oi;

import static edu.wpi.first.units.Units.Degrees;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.turret.TurretMap;
import frc.robot.subsystems.turret.Turret.OperatingMode;
import frc.robot.Container;
import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;
import frc.robot.subsystems.hopper.Hopper.TransferFeedState;

public class OperatorInterface {
        public static class OIMap {
                public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Drone;
        }

        public SupplierXboxController DriverController;
        public SupplierXboxController OperatorController;

        public OperatorInterface() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        public void bindDriverControls() {
                // Driver Controls:
                // Swerve controls with sticks
                // RB - Face away from hub
                // A - Reset gyro
                // X - Intake out position
                // Y - Intake in position
                // LT - Intake feed in
                // Start + Left d-pad - Climb setup
                // Start + Up d-pad - Start climb
                // Start + Right d-pad - Reset climb
                // Start + Down d-pad - End climb

                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                // Serve controls
                Container.Swerve.setDefaultCommand(Container.Swerve.driveFieldRelativeCommand(controlProfile));

                // Face away from hub
                DriverController.rightBumper().whileTrue(Container.Swerve.faceAwayFromHubCommand());

                // Disabled, untested and not tuned
                // DriverController.x()
                //                 .onTrue(Container.Swerve.disableAutoAlignCommand());

                // Reset gyro
                DriverController.a()
                                .onTrue(Container.Swerve.resetGyroCommand());

                // Intake position control
                DriverController.x().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT));
                DriverController.y().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.IN));

                // Intake feed control
                DriverController.leftTrigger().onTrue(Container.Hopper.setFeed(TransferFeedState.INWARDS));
                DriverController.leftTrigger().onFalse(Container.Hopper.setFeed(TransferFeedState.OUTWARDS));

                // Combined climb controls
                DriverController.start().and(DriverController.povLeft())
                                .onTrue(Container.setupClimb());
                DriverController.start().and(DriverController.povUp())
                                .onTrue(Container.startClimbing());
                DriverController.start().and(DriverController.povDown())
                                .onTrue(Container.stopClimbing());
                DriverController.start().and(DriverController.povRight())
                                .onTrue(Container.resetRobotAfterClimb());

                // -------------------------- TEST COMMANDS --------------------------

                // These commands are for testing the functionality of specific subsystems. When using, comment out
                // all other driver controls and uncomment the controls for the subsystem below that you would like to test.

                // DriverController.leftTrigger().onTrue(Container.Hopper.setIntakeFeed(IntakeFeedState.INWARDS));
                // DriverController.leftTrigger().onFalse(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED));

                // DriverController.rightBumper().onTrue(Container.Hopper.setIntakeFeed(IntakeFeedState.OUTWARDS));
                // DriverController.rightBumper().onFalse(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED));

                // DriverController.a().onTrue(Container.Hopper.setFeed(TransferFeedState.INWARDS));
                // DriverController.a().onFalse(Container.Hopper.setFeed(TransferFeedState.STOPPED));

                // DriverController.b().onTrue(Container.Hopper.setFeed(TransferFeedState.OUTWARDS));
                // DriverController.b().onFalse(Container.Hopper.setFeed(TransferFeedState.STOPPED));

                // DriverController.a().onTrue(Container.Turret.setFeed(UptakeState.FORWARDS));
                // DriverController.a().onFalse(Container.Turret.setFeed(UptakeState.STOPPED));

                // DriverController.b().onTrue(Container.Turret.setFeed(UptakeState.REVERSED));
                // DriverController.b().onFalse(Container.Turret.setFeed(UptakeState.STOPPED));

                // DriverController.x().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT));
                // DriverController.y().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.IN));

                // DriverController.a().onTrue(Container.Turret.setFlywheel(FlywheelState.IDLE));
                // DriverController.a().onFalse(Container.Turret.setFlywheel(FlywheelState.STOPPED));

                // DriverController.a().onTrue(Container.Climb.setSupport(SupportState.RAISED));
                // DriverController.b().onTrue(Container.Climb.setSupport(SupportState.LOWERED));

                // -- CLIMB TEST COMMANDS --

                // Use these to manually test climb functionality before moving on to combined Commands
                // After all subsystems arer shown to work, test the normal combined climb Commands

                // DriverController.a().onTrue(Container.Climb.setClimb(ClimbState.UP));
                // DriverController.a().onFalse(Container.Climb.setClimb(ClimbState.STOPPED));

                // DriverController.b().onTrue(Container.Climb.setClimb(ClimbState.DOWN));
                // DriverController.b().onFalse(Container.Climb.setClimb(ClimbState.STOPPED));

                // DriverController.x().onTrue(Container.Climb.setBrake(FrictionBrakeState.APPLIED));
                // DriverController.y().onTrue(Container.Climb.setBrake(FrictionBrakeState.RELEASED));

                // -------------------------

                // DriverController.povUp().whileTrue(
                //                 Container.Turret.sysIdFlywheelCommand(TestType.DYNAMIC, TestDirection.FORWARD));
                // DriverController.povDown().whileTrue(
                //                 Container.Turret.sysIdFlywheelCommand(TestType.DYNAMIC, TestDirection.REVERSE));
                // DriverController.y().whileTrue(
                //                 Container.Turret.sysIdFlywheelCommand(TestType.QUASISTATIC, TestDirection.FORWARD));
                // DriverController.a().whileTrue(
                //                 Container.Turret.sysIdFlywheelCommand(TestType.QUASISTATIC, TestDirection.REVERSE));
        }

        public void bindOperatorControls() {
                // Operator Controls:
                // Up d-pad - Turret hood up
                // Down d-pad - Turret hood down
                // Left d-pad - Turret yaw left
                // Right d-pad - Turret yaw right
                // X - Flywheel up
                // A - Flywheel down
                // Y - 100% intake power while held
                // RT - Shoot
                // LT - Set turret auto mode
                // LB - Set turret manual mode

                // Fire fuel
                OperatorController.rightTrigger()
                                .onTrue(Container.startShooting())
                                .onFalse(Container.stopShooting());

                // Controls to toggle Turret auto and manual
                // OperatorController.leftTrigger()
                //                 .onTrue(Container.Turret.setOperatingMode(OperatingMode.AUTO));
                // OperatorController.leftBumper()
                //                 .onTrue(Container.Turret.setOperatingMode(OperatingMode.MANUAL));

                // Manual turret controls
                // OperatorController.povRight()
                //                 .onTrue(Container.Turret.adjustManualYaw(TurretMap.MANUAL_YAW_STEP_DEGREES));
                // OperatorController.povLeft()
                //                 .onTrue(Container.Turret.adjustManualYaw(-TurretMap.MANUAL_YAW_STEP_DEGREES));
                // OperatorController.povUp()
                //                 .onTrue(Container.Turret.adjustManualHoodAngle(TurretMap.MANUAL_HOOD_STEP_DEGREES));
                // OperatorController.povDown()
                //                 .onTrue(Container.Turret.adjustManualHoodAngle(-TurretMap.MANUAL_HOOD_STEP_DEGREES));
                OperatorController.x()
                                .onTrue(Container.Turret
                                                .adjustManualFlywheelSpeed(TurretMap.MANUAL_FLYWHEEL_STEP_RPS));
                OperatorController.a()
                                .onTrue(Container.Turret
                                                .adjustManualFlywheelSpeed(-TurretMap.MANUAL_FLYWHEEL_STEP_RPS));

                // Control intake feed percent out
                OperatorController.y().whileTrue(Container.Hopper.overrideIntakeFeedPercentOut(1));
        }

        public void setControllerRumbleIntensity(SupplierXboxController controller, double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

        public Command rumbleControllerShort(SupplierXboxController controller) {
                return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1))
                                .andThen(Commands.waitSeconds(0.2))
                                .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
        }
}
