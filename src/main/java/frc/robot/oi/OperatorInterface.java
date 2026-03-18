package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import org.prime.sysid.SysIdRoutineHelper.TestDirection;
import org.prime.sysid.SysIdRoutineHelper.TestType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.turret.Turret.FiringState;
import frc.robot.subsystems.turret.Turret.UptakeState;
import frc.robot.subsystems.turret.TurretMap;
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

        private double _manualTurretPercentOut = 0.1;
        private IntakeFeedState _automaticFeedState = IntakeFeedState.INWARDS;

        public OperatorInterface() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        public void bindDriverControls() {
                // Driver Controls:
                // Swerve controls with sticks
                // RB - Face away from hub
                // A - Reset gyro
                // Up d-pad - SysId quasistatic forward
                // Down d-pad - SysId quasistatic reverse
                // Right d-pad - SysId dynamic forward
                // Left d-pad - SysId dynamic reverse

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
                //                 .onTrue(Container.Swerve.disableAutoAlignCommand());

                // Intake feed control
                // DriverController.leftTrigger().onTrue(Container.Hopper.setIntakeFeed(IntakeFeedState.INWARDS));
                // DriverController.leftTrigger().onFalse(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED));

                // SysId drive characterization routines
                DriverController.povUp()
                                .whileTrue(Container.Swerve.sysIdDriveCommand(TestType.QUASISTATIC,
                                                TestDirection.FORWARD))
                                .onFalse(Container.Swerve.stopAllMotorsCommand());
                DriverController.povDown()
                                .whileTrue(Container.Swerve.sysIdDriveCommand(TestType.QUASISTATIC,
                                                TestDirection.REVERSE))
                                .onFalse(Container.Swerve.stopAllMotorsCommand());
                DriverController.povRight()
                                .whileTrue(Container.Swerve.sysIdDriveCommand(TestType.DYNAMIC, TestDirection.FORWARD))
                                .onFalse(Container.Swerve.stopAllMotorsCommand());
                DriverController.povLeft()
                                .whileTrue(Container.Swerve.sysIdDriveCommand(TestType.DYNAMIC, TestDirection.REVERSE))
                                .onFalse(Container.Swerve.stopAllMotorsCommand());

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
        }

        public void bindOperatorControls() {
                // Operator Controls:
                // RT - Shoot (uptake + firing)
                // RB - Transfer feed in
                // Y - Intake out position + auto feed
                // B - Intake in position + stop feed after 1.5s
                // LB - Outtake (intake + transfer outwards)
                // Up d-pad - Flywheel speed up
                // Down d-pad - Flywheel speed down
                // Left d-pad - Turret yaw left
                // Right d-pad - Turret yaw right
                // Start - 100% intake feed override

                OperatorController.rightTrigger()
                                .onTrue(Container.Turret.setFeed(UptakeState.FORWARDS)
                                                .andThen(Container.Turret.setFiring(FiringState.FIRING)))
                                .onFalse(Container.Turret.setFeed(UptakeState.STOPPED)
                                                .andThen(Container.Turret.setFiring(FiringState.IDLE))
                                                .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));

                OperatorController.rightBumper().onTrue(Container.Hopper.setFeed(TransferFeedState.INWARDS))
                                .onFalse(Container.Hopper.setFeed(TransferFeedState.STOPPED));

                // Intake position control
                OperatorController.y().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT)
                                .andThen(Container.Hopper.setIntakeFeedSupplier(() -> _automaticFeedState)));
                OperatorController.b().onTrue(
                                Container.Hopper.setHopperIntakeControl(HopperIntakeState.IN)
                                                .andThen(Commands.waitSeconds(1.5))
                                                .andThen(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED))
                                                .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));

                // Controls to toggle Turret auto and manual
                // OperatorController.leftTrigger()
                //                 .onTrue(Container.Turret.setOperatingMode(OperatingMode.AUTO));
                // OperatorController.leftBumper()
                //                 .onTrue(Container.Turret.setOperatingMode(OperatingMode.MANUAL));

                // Manual turret controls
                OperatorController.povRight()
                                .onTrue(Container.Turret.adjustManualYaw(TurretMap.MANUAL_YAW_STEP_DEGREES));
                OperatorController.povLeft()
                                .onTrue(Container.Turret.adjustManualYaw(-TurretMap.MANUAL_YAW_STEP_DEGREES));
                OperatorController.povUp()
                                .onTrue(Container.Turret
                                                .adjustManualFlywheelSpeed(TurretMap.MANUAL_FLYWHEEL_STEP_RPS));
                OperatorController.povDown()
                                .onTrue(Container.Turret
                                                .adjustManualFlywheelSpeed(-TurretMap.MANUAL_FLYWHEEL_STEP_RPS));

                // Control intake feed percent out
                OperatorController.start().whileTrue(Container.Hopper.overrideIntakeFeedPercentOut(1));

                // Outtake
                OperatorController.leftBumper().onTrue(Container.Hopper.setIntakeFeed(IntakeFeedState.OUTWARDS)
                                .andThen(Container.Hopper.setFeed(TransferFeedState.OUTWARDS)))
                                .onFalse(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED)
                                                .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));
        }

        public void setControllerRumbleIntensity(SupplierXboxController controller, double intensity) {
                controller.setRumble(RumbleType.kBothRumble, intensity);
        }

        public Command rumbleControllerShort(SupplierXboxController controller) {
                return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1))
                                .andThen(Commands.waitSeconds(0.2))
                                .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
        }

        public Command changeFlywheelSpeed(double change) {
                return Commands.runOnce(() -> {
                        var value = _manualTurretPercentOut + change;
                        System.out.println(value);
                        if (value >= 1) {
                                _manualTurretPercentOut = 1;
                        } else if (value <= -1) {
                                _manualTurretPercentOut = -1;
                        } else {
                                _manualTurretPercentOut = value;
                        }

                        System.out.println(_manualTurretPercentOut);
                });
        }
}
