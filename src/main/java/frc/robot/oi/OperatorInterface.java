package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.turret.Turret;
import frc.robot.Container;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.LimelightNameEnum;
import frc.robot.subsystems.vision.Vision;

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

        public void bindDriverControls(Swerve swerve, Vision vision, Turret turret, Climb climb, Hopper hopper) {
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
                DriverController.start().and(DriverController.pov(Controls.up)).onTrue(Container.setupClimb());
                DriverController.start().and(DriverController.pov(Controls.left)).onTrue(Container.startClimbing());
                DriverController.start().and(DriverController.pov(Controls.down)).onTrue(Container.stopClimbing());
                DriverController.start().and(DriverController.pov(Controls.right))
                                .onTrue(Container.resetRobotAfterClimb());

                DriverController.x().and(DriverController.pov(Controls.up)).onTrue(Container.Hopper.setIntakeOut());
                DriverController.x().and(DriverController.pov(Controls.down)).onTrue(Container.Hopper.setIntakeIn());

        }

        public void bindOperatorControls(Swerve swerve, Vision vision, Turret turret, Climb climb,
                        Hopper hopper) {
                // Changes the vision mode for the rear limelight. 
                OperatorController.start()
                                .onTrue(vision.setLimelightPipeline(LimelightNameEnum.kRear, 1))
                                .onFalse(vision.setLimelightPipeline(LimelightNameEnum.kRear, 0));

                OperatorController.rightTrigger()
                                .onTrue(Container.startShooting())
                                .onFalse(Container.stopShooting());
                // Right joystick to aim turret, left joystick to move hood
                Container.Turret.setYawSupplier(OperatorController.getRightStickXSupplier(0.05));
                Container.Turret.setPitchSupplier(OperatorController.getLeftStickYSupplier(0.05));
                // Controls to toggle auto and manual
                OperatorController.start().and(OperatorController.pov(Controls.up))
                                .onTrue(Container.Turret.setTargetingAuto());
                OperatorController.start().and(OperatorController.pov(Controls.down))
                                .onTrue(Container.Turret.setTargetingManual());

        }

        public void setDriverRumbleIntensity(double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

}
