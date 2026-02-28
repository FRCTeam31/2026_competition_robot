package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TargetingState;
import frc.robot.Container;
import frc.robot.Container.IntakeCombinedState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperIntakeState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightVision;

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

        public void bindDriverControls(Swerve swerve, LimelightVision vision, Turret turret, Climb climb,
                        Hopper hopper) {
                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                swerve.setDefaultCommand(swerve.driveFieldRelativeCommand(controlProfile));

                DriverController.x()
                                .onTrue(swerve.disableAutoAlignCommand());
                DriverController.a()
                                .onTrue(swerve.resetGyroCommand());

                // While holding POV up, auto-align the robot to the in-view apriltag target's rotation
                DriverController.pov(Controls.up)
                                .onTrue(swerve.disableAutoAlignCommand());

                // TODO: Add rumble and light feedback to different climb states
                // TODO: Figure out why the rumble is not WORKING
                // TODO: Test full commands
                // TODO: Add distance checks to climb
                DriverController.start().and(DriverController.pov(Controls.up))
                                .onTrue(Container.setupClimb()); //.andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.left))
                                .onTrue(Container.startClimbing()); //.andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.down))
                                .onTrue(Container.stopClimbing()); //.andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.right))
                                .onTrue(Container.resetRobotAfterClimb()); //.andThen(rumbleControllerShort(DriverController)));

                DriverController.x().and(DriverController.pov(Controls.up))
                                .onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.OUT));
                DriverController.x().and(DriverController.pov(Controls.down))
                                .onTrue(Container.Hopper.setHopperIntakeControl(HopperIntakeState.IN));

                DriverController.start().and(DriverController.rightBumper()).and(DriverController.leftBumper())
                                .onTrue(Container.setIntakeStates(IntakeCombinedState.OUTWARDS))
                                .onFalse(Container.setIntakeStates(IntakeCombinedState.INWARDS));

                // -------------------------- TEST COMMANDS --------------------------

                // These commands are for testing the functionality of specific subsystems. When using, comment out
                // all other driver controls and uncomment the controls for the subsystem below that you would like to test.

                // DriverController.a().onTrue(Container.Hopper.setIntakeFeed(IntakeFeedState.INWARDS));
                // DriverController.a().onFalse(Container.Hopper.setIntakeFeed(IntakeFeedState.STOPPED));

                // DriverController.a().onTrue(Container.Hopper.setFeed(TransferFeedState.INWARDS));
                // DriverController.a().onFalse(Container.Hopper.setFeed(TransferFeedState.STOPPED));

                // DriverController.a().onTrue(Container.Turret.setFeed(UptakeState.FORWARDS));
                // DriverController.a().onFalse(Container.Turret.setFeed(UptakeState.STOPPED));

                // DriverController.a().onTrue(Container.Turret.setFlywheel(FlywheelState.IDLE));
                // DriverController.a().onFalse(Container.Turret.setFlywheel(FlywheelState.STOPPED));

                // DriverController.a().onTrue(Container.Climb.setSupport(SupportState.RAISED));
                // DriverController.b().onTrue(Container.Climb.setSupport(SupportState.LOWERED));

                // DriverController.a().onTrue(Container.Climb.setBrake(FrictionBrakeState.APPLIED));
                // DriverController.b().onTrue(Container.Climb.setBrake(FrictionBrakeState.RELEASED));

                // -------------------------------------------------------------------

        }

        public void bindOperatorControls(Swerve swerve, LimelightVision vision, Turret turret, Climb climb,
                        Hopper hopper) {
                // Changes the vision mode for the turret limelight. 
                OperatorController.start()
                                .onTrue(vision.setProcessingPipeline(VisionMap.LimelightTurretName, 1))
                                .onFalse(vision.setProcessingPipeline(VisionMap.LimelightTurretName, 0));

                OperatorController.rightTrigger()
                                .onTrue(Container.startShooting())
                                .onFalse(Container.stopShooting());
                // Right joystick to aim turret, left joystick to move hood
                Container.Turret.setYawSupplier(OperatorController.getRightStickXSupplier(0.05));
                Container.Turret.setPitchSupplier(OperatorController.getLeftStickYSupplier(0.05));

                // Controls to toggle auto and manual
                OperatorController.start().and(OperatorController.pov(Controls.up))
                                .onTrue(Container.Turret.setTargeting(TargetingState.AUTO));
                OperatorController.start().and(OperatorController.pov(Controls.down))
                                .onTrue(Container.Turret.setTargeting(TargetingState.MANUAL));

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
