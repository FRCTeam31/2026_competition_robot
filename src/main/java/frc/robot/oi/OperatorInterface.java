package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.swerve.SwerveMap;
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

        public void bindDriverControls(Swerve swerve, Vision vision) {
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

                // Changes the vision mode for the rear limelight. 
                OperatorController.start()
                                .onTrue(vision.setLimelightPipeline(LimelightNameEnum.kRear, 1))
                                .onFalse(vision.setLimelightPipeline(LimelightNameEnum.kRear, 0));
        }

        public void bindOperatorControls(Swerve swerveSubsystem, Vision visionSubsystem) {

        }

        public void setDriverRumbleIntensity(double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

}
