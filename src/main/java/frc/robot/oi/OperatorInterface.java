package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.turret.Turret.FiringState;
import frc.robot.subsystems.turret.Turret.OperatingMode;
import frc.robot.subsystems.turret.Turret.UptakeState;
import frc.robot.subsystems.turret.TurretMap;
import frc.robot.Container;
import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;
import frc.robot.subsystems.hopper.Hopper.TransferFeedState;

public class OperatorInterface {
        public static class OIMap {
                // public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Drone;
                public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.DroneReversed;
        }

        public SupplierXboxController DriverController;
        public SupplierXboxController OperatorController;

        private IntakeFeedState _automaticFeedState = IntakeFeedState.INWARDS;

        public OperatorInterface() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        public void bindDriverControls() {
                // Driver Controls:
                // Swerve controls with sticks
                // A - Reset gyro

                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                // Serve controls
                Container.Swerve.setDefaultCommand(Container.Swerve.driveFieldRelativeCommand(controlProfile));

                // Reset gyro
                DriverController.a()
                                .onTrue(Container.Swerve.resetGyroCommand());
        }

        public void bindOperatorControls() {
                // Operator Controls:
                // RT - Shoot (uptake + firing + transfer)
                // RB - Transfer feed in
                // LB - Set turret auto
                // Y - Intake out position + auto feed
                // B - Intake in position + stop feed after 1.5s
                // X - E-Stop intake rollers
                // Up d-pad - Flywheel speed up / manual mode
                // Down d-pad - Flywheel speed down / manual mode
                // Left d-pad - Turret yaw CCW / manual mode
                // Right d-pad - Turret yaw CW / manual mode

                OperatorController.rightTrigger()
                                .onTrue(Container.Turret.setFeed(UptakeState.FORWARDS)
                                                .andThen(Container.Turret.setFiring(FiringState.FIRING))
                                                .andThen(Container.Hopper.setFeed(TransferFeedState.INWARDS)))
                                // .andThen(Container.Hopper.oscillateIntake(HopperMap.INTAKE_OSCILLATION_CYCLE_SECONDS)))
                                .onFalse(Container.Turret.setFeed(UptakeState.STOPPED)
                                                .andThen(Container.Turret.setFiring(FiringState.FIRING))
                                                .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));

                OperatorController.rightBumper().onTrue(Container.Hopper.setFeed(TransferFeedState.INWARDS))
                                .onFalse(Container.Hopper.setFeed(TransferFeedState.STOPPED));

                OperatorController.y().onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT)
                                .andThen(Container.Hopper.setIntakeFeedSupplier(() -> _automaticFeedState)));
                OperatorController.b().onTrue(
                                Container.Hopper.setHopperIntakeControl(HopperIntakeState.IN)
                                                .andThen(Commands.waitSeconds(1.5))
                                                .andThen(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED))
                                                .andThen(Container.Hopper.setFeed(TransferFeedState.STOPPED)));

                OperatorController.x().onTrue(Commands
                                .runOnce(() -> _automaticFeedState = _automaticFeedState != IntakeFeedState.STOPPED
                                                ? IntakeFeedState.STOPPED
                                                : IntakeFeedState.INWARDS));

                // Controls to toggle Turret auto and manual
                OperatorController.leftBumper()
                                .onTrue(Container.Turret.setOperatingMode(OperatingMode.AUTO));

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
        }

        public void setControllerRumbleIntensity(SupplierXboxController controller, double intensity) {
                controller.setRumble(RumbleType.kBothRumble, intensity);
        }

        public Command rumbleControllerShort(SupplierXboxController controller) {
                return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1))
                                .andThen(Commands.waitSeconds(0.2))
                                .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
        }
}
