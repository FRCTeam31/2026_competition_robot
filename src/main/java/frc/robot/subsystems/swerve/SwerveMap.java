package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;

import frc.robot.subsystems.swerve.module.SwerveModuleMap;

import org.prime.control.ExtendedPIDConstants;

public class SwerveMap {
        public class Chassis {
                public static final double TrackWidthMeters = Units.inchesToMeters(24.75);
                public static final double WheelBaseMeters = Units.inchesToMeters(24.75);
                public static final double MaxSpeedMetersPerSecond = 5.4;
                public static final double MaxAngularSpeedRadians = Math.PI * 2;
                public static final double BumperWidthMeters = Units.inchesToMeters(2.5);
                public static final double ApproachDistance = (WheelBaseMeters / 2) + BumperWidthMeters - 0.1;
        }

        public class Control {
                public static final double DriveDeadband = 0.15;
                public static final double DeadbandCurveWeight = 0.5;
        }

        // PID Constants
        public static final ExtendedPIDConstants DrivePID = new ExtendedPIDConstants(0.0075, 0, 0.000, 0.0, 0.099, 0.12,
                        0.14);
        public static final ExtendedPIDConstants SteeringPID = new ExtendedPIDConstants(4.1, 0, 0.04);
        public static final ExtendedPIDConstants AutoAlignPID = new ExtendedPIDConstants(4, 0, 0.08);
        public static final ExtendedPIDConstants PathPlannerTranslationPID = new ExtendedPIDConstants(4.5, 0, 0);
        public static final ExtendedPIDConstants PathPlannerRotationPID = new ExtendedPIDConstants(2, 0, 0);

        // Uniform Drive Constants
        public static final double DriveGearRatio = 6.75;
        public static final double DriveWheelDiameterMeters = Units.inchesToMeters(3.875);
        public static final double DriveWheelCircumferenceMeters = Math.PI * DriveWheelDiameterMeters;
        public static final double SteeringGearRatio = 18.75;

        // TODO: Reevaluate these constants after testing drive current limits on robot
        public static final int DriveStallCurrentLimit = 40;
        public static final int DriveFreeCurrentLimit = 30;

        public static final int DriveSupplyCurrentLimitDuration = 100;

        public static final int PigeonId = 13;
        public static final SwerveModuleMap FrontLeftSwerveModule = new SwerveModuleMap(
                        3,
                        4,
                        10,
                        0.563721,
                        true,
                        false,
                        new Translation2d(Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));
        public static final SwerveModuleMap FrontRightSwerveModule = new SwerveModuleMap(
                        6,
                        5,
                        11,
                        0.540039,
                        true,
                        false,
                        new Translation2d(Chassis.TrackWidthMeters / 2, -(Chassis.WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearRightSwerveModule = new SwerveModuleMap(
                        7,
                        8,
                        12,
                        0.186279,
                        false,
                        false,
                        new Translation2d(-(Chassis.TrackWidthMeters / 2), -(Chassis.WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearLeftSwerveModule = new SwerveModuleMap(
                        2,
                        1,
                        9,
                        0.075928,
                        false,
                        false,
                        new Translation2d(-Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));

        public static final RobotConfig PathPlannerRobotConfiguration = new RobotConfig(
                        // TODO: Determine the robot's weight and MOI
                        Units.lbsToKilograms(50),
                        MomentOfInertia.ofBaseUnits(3, edu.wpi.first.units.Units.KilogramSquareMeters)
                                        .baseUnitMagnitude(),
                        new ModuleConfig(
                                        DriveWheelDiameterMeters / 2,
                                        Chassis.MaxSpeedMetersPerSecond / 10,
                                        1.0,
                                        DCMotor.getNeoVortex(1).withReduction(DriveGearRatio),
                                        DriveStallCurrentLimit,
                                        1),
                        FrontLeftSwerveModule.ModuleLocation,
                        FrontRightSwerveModule.ModuleLocation,
                        RearLeftSwerveModule.ModuleLocation,
                        RearRightSwerveModule.ModuleLocation);
}
