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
                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                Container.Swerve.setDefaultCommand(Container.Swerve.driveFieldRelativeCommand(controlProfile));

                DriverController.y().onTrue(Container.Swerve.faceAwayFromHubCommand());
                DriverController.x()
                                .onTrue(Container.Swerve.disableAutoAlignCommand());
                DriverController.a()
                                .onTrue(Container.Swerve.resetGyroCommand());

                // TODO: Add rumble and light feedback to different climb states
                // TODO: Test full commands
                // TODO: Add distance checks to climb
                DriverController.start().and(DriverController.pov(Controls.up))
                                .onTrue(Container.setupClimb().andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.left))
                                .onTrue(Container.startClimbing().andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.down))
                                .onTrue(Container.stopClimbing().andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.right))
                                .onTrue(Container.resetRobotAfterClimb()
                                                .andThen(rumbleControllerShort(DriverController)));

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
                // Fire fuel
                OperatorController.rightTrigger()
                                .onTrue(Container.startShooting())
                                .onFalse(Container.stopShooting());

                // Controls to toggle Turret auto and manual
                OperatorController.start().and(OperatorController.pov(Controls.up))
                                .onTrue(Container.Turret.setOperatingMode(OperatingMode.AUTO));
                OperatorController.start().and(OperatorController.pov(Controls.down))
                                .onTrue(Container.Turret.setOperatingMode(OperatingMode.MANUAL));

                // Manual turret controls
                OperatorController.pov(90)
                                .onTrue(Container.Turret.adjustManualYaw(TurretMap.MANUAL_YAW_STEP_DEGREES));
                OperatorController.pov(270)
                                .onTrue(Container.Turret.adjustManualYaw(-TurretMap.MANUAL_YAW_STEP_DEGREES));
                OperatorController.pov(0)
                                .onTrue(Container.Turret.adjustManualHoodAngle(TurretMap.MANUAL_HOOD_STEP_DEGREES));
                OperatorController.pov(180)
                                .onTrue(Container.Turret.adjustManualHoodAngle(-TurretMap.MANUAL_HOOD_STEP_DEGREES));
                OperatorController.x()
                                .onTrue(Container.Turret
                                                .adjustManualFlywheelSpeed(-TurretMap.MANUAL_FLYWHEEL_STEP_RPS));
                OperatorController.y()
                                .onTrue(Container.Turret.adjustManualFlywheelSpeed(TurretMap.MANUAL_FLYWHEEL_STEP_RPS));

                // Intake: arm out + rollers inward or arm in and rollers stopped
                OperatorController.leftBumper()
                                .onTrue(Container.Hopper.toggleHopperIntake());
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
