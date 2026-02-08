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
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.ExtensionState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionMap;

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
                DriverController.start().and(DriverController.pov(Controls.up))
                                .onTrue(Container.setupClimb()); //.andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.left))
                                .onTrue(Container.startClimbing()); //.andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.down))
                                .onTrue(Container.stopClimbing()); //.andThen(rumbleControllerShort(DriverController)));
                DriverController.start().and(DriverController.pov(Controls.right))
                                .onTrue(Container.resetRobotAfterClimb()); //.andThen(rumbleControllerShort(DriverController)));

                DriverController.x().and(DriverController.pov(Controls.up))
                                .onTrue(Container.Hopper.setHopper(ExtensionState.OUT));
                DriverController.x().and(DriverController.pov(Controls.down))
                                .onTrue(Container.Hopper.setHopper(ExtensionState.IN));

                DriverController.start().and(DriverController.rightBumper()).and(DriverController.leftBumper())
                                .onTrue(Container.setIntakeStates(false)).onFalse(Container.setIntakeStates(true));

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
