package frc.robot.subsystems.turret;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import edu.wpi.first.units.measure.Distance;
import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretMap {
        // ------------------------ Feature Flags -------------------------
        public static final boolean UPDATE_LIMELIGHT_POSE = true;
        public static final boolean AUTO_MOTION_COMPENSATION = false;
        public static final boolean USE_SPEED_INTERPOLATION = false;
        public static final boolean YAW_DEADZONE_ENABLED = true;
        public static final boolean USE_LIMELIGHT_TARGETING = false;
        public static final boolean USE_LIMELIGHT_YAW_CORRECTION = false;

        // ------------------------ Turret Yaw ----------------------------
        public static final int TURRET_ROTATOR_CANID = 17;
        public static final boolean TURRET_ROTATOR_INVERTED = false;
        public static final double TURRET_GEAR_RATIO = 10;
        public static final ExtendedPIDConstants TURRET_ROTATOR_PID = new ExtendedPIDConstants(5.5, 0, 0);
        public static final double YAW_MOTION_MAGIC_CRUISE_VELOCITY = 2000; // sensor units per 100ms
        public static final double YAW_MOTION_MAGIC_ACCELERATION = 4000; // sensor units per 100ms per second
        public static final double YAW_MAX_MANUAL_PERCENT_OUT = 1;
        public static final double AUTO_AIM_YAW_TRIM_DEGREES = 10;
        public static final double TURRET_CORRECTION_THRESHOLD_DEGREES = 2.0;
        public static final double YAW_ON_TARGET_TOLERANCE_DEGREES = 2.0;
        public static final Angle YAW_RESET_ANGLE = Degrees.of(180);
        public static final int TURRET_RESET_SWITCH_CHANNEL = 0;

        // ------------------------ Turret Dead Zone ----------------------
        // The arc from DEADZONE_START to DEADZONE_END (going clockwise
        // through the larger angle) that the turret physically cannot traverse.
        // Both values are in degrees, measured in the same frame as turret rotation (0-360).
        // Example: START = 170, END = 190 means a 20-degrees dead zone centered on 180-degrees.
        public static final double DEADZONE_START_DEGREES = 285;
        public static final double DEADZONE_END_DEGREES = 75;

        // ------------------------ Turret Geometry -----------------------
        public static final Rotation2d TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT = new Rotation2d()
                        .rotateBy(Rotation2d.kCCW_90deg);
        public static final double TURRET_HEIGHT_ABOVE_GROUND = 0.45;
        public static Translation3d TURRET_ROBOT_ORIGIN = new Translation3d(
                        Units.inchesToMeters(8.25),
                        Units.inchesToMeters(5.75),
                        Units.inchesToMeters(15.894));
        public static final double TURRET_DISTANCE_FROM_ROBOT_CENTER = Math.hypot(TURRET_ROBOT_ORIGIN.getX(),
                        TURRET_ROBOT_ORIGIN.getY());

        // ------------------------ Flywheel ------------------------------
        public static final int FLYWHEEL_LEFT_CANID = 19;
        public static final int FLYWHEEL_RIGHT_CANID = 20;
        public static final boolean FLYWHEEL_LEFT_INVERTED = false;
        public static final double FLYWHEEL_RAMP_PERIOD = 0.05;
        public static final ExtendedPIDConstants FLYWHEEL_PID = new ExtendedPIDConstants(
                        0,
                        0,
                        0,
                        0,
                        0.1645,
                        0.013876,
                        0.075161);
        public static final AngularVelocity FLYWHEEL_IDLE_VELOCITY = RotationsPerSecond.of(45);
        public static final double FLYWHEEL_RADIUS = 0.0505;
        public static final double FLYWHEEL_MAX_SPEED_RPS = 90;
        public static final double FLYWHEEL_MIN_SPEED_RPS = 0.0;
        public static final double FLYWHEEL_AT_SPEED_TOLERANCE_PERCENT = 5.0; // 5% tolerance

        // Distance in meters and flywheel speed in RPM
        public static double MAX_HOOD_MIN_DIST_METERS = Units.inchesToMeters(47.5);
        public static double MAX_HOOD_MAX_DIST_METERS = Units.inchesToMeters(140.5);
        public static final InterpolatingDoubleTreeMap TARGET_DIST_FLYSPEED_RPS_MAX_HOOD_MAP = InterpolatingDoubleTreeMap
                        .ofEntries(Map.entry(MAX_HOOD_MIN_DIST_METERS, RPM.of(2700).in(RotationsPerSecond)),
                                        Map.entry(Units.inchesToMeters(97.5), RPM.of(3200).in(RotationsPerSecond)),
                                        Map.entry(Units.inchesToMeters(109.5), RPM.of(3500).in(RotationsPerSecond)),
                                        Map.entry(MAX_HOOD_MAX_DIST_METERS, RPM.of(3700).in(RotationsPerSecond)));

        public static double MIN_HOOD_MIN_DIST_METERS = Units.inchesToMeters(72);
        public static double MIN_HOOD_MAX_DIST_METERS = Units.inchesToMeters(239);
        public static final InterpolatingDoubleTreeMap TARGET_DIST_FLYSPEED_RPS_MIN_HOOD_MAP = InterpolatingDoubleTreeMap
                        .ofEntries(Map.entry(MIN_HOOD_MIN_DIST_METERS, RPM.of(2000).in(RotationsPerSecond)),
                                        Map.entry(Units.inchesToMeters(108), RPM.of(2600).in(RotationsPerSecond)),
                                        Map.entry(Units.inchesToMeters(152), RPM.of(3100).in(RotationsPerSecond)),
                                        Map.entry(Units.inchesToMeters(213), RPM.of(3700).in(RotationsPerSecond)),
                                        Map.entry(MIN_HOOD_MAX_DIST_METERS, RPM.of(4200).in(RotationsPerSecond)));

        // ------------------------ Hood -----------------------------------
        public static final int HOOD_CAN_ID = 18;
        public static final boolean HOOD_INVERTED = false;
        public static final double HOOD_MAX_ANGLE_DEGREES = 35.1; // Hood fully retracted
        public static final double HOOD_HALF_ANGLE_DEGREES = 23.85;
        // public static final double HOOD_MIN_ANGLE_DEGREES = 12.6; // Hood fully extended
        public static final double HOOD_MIN_ANGLE_DEGREES = HOOD_MAX_ANGLE_DEGREES; // Hood fully extended
        public static final double HOOD_ANGLE_RANGE_DEGREES = HOOD_MAX_ANGLE_DEGREES - HOOD_MIN_ANGLE_DEGREES;
        public static final ExtendedPIDConstants HOOD_PID = new ExtendedPIDConstants(0.1, 0, 0);
        public static final double HOOD_ON_TARGET_TOLERANCE_DEGREES = 1.0;
        public static final Distance HOOD_GEAR_RADIUS = Millimeters.of(10);
        public static final AngularVelocity HOOD_SIM_MAX_SPEED = RadiansPerSecond.of(183.33 * Math.PI * 2);
        public static final double HOOD_GEAR_RATIO = 6.333;
        public static final int HOOD_SERVO_CHANNEL = 9;

        // ------------------------ Feeder ---------------------------------
        public static final int FEEDER_CANID = 16;
        public static final boolean FEEDER_INVERTED = false;
        public static final double MAX_FEED_PERCENT_OUT = 0.4;
        public static final double FEEDER_VELOCITY_CONVERSION_FACTOR = 1.0;

        // ------------------------ Manual Mode Steps ----------------------
        /** Amount to adjust the flywheel speed per button press in MANUAL mode (RPS) */
        public static final double MANUAL_FLYWHEEL_STEP_RPS = 2.0;
        /** Amount to adjust the hood angle per button press in MANUAL mode (degrees) */
        public static final double MANUAL_HOOD_STEP_DEGREES = 1.0;
        /** Amount to adjust the turret yaw per button press in MANUAL mode (degrees) */
        public static final double MANUAL_YAW_STEP_DEGREES = 30.0;

        // ------------------------ Home Positions -------------------------
        /** Yaw home position in degrees (180-degrees = straight back) */
        public static final double YAW_HOME_DEGREES = 180.0;
        /** Hood home angle in degrees (fully up / hood lowered) */
        public static final double HOOD_HOME_DEGREES = HOOD_MAX_ANGLE_DEGREES;

        // ------------------------ Shot Targeting -------------------------
        // public static final Pose3d HUB_GOAL_POSITION = new Pose3d();
        public static final double HUB_OVERSHOOT_HEIGHT = 0.2;
        public static final double MIN_SHOT_DISTANCE_METERS = 0;
        public static final double FLYWHEEL_GEAR_RATIO = 32d / 50d; // 32t on motor gear, 50t on flywheel gear
        public static final double TURRET_YAW_MANUAL_SENSITIVITY = 30 / 50;

        // ------------------------ Limelight ------------------------------
        // Offset from turret rotation center (in meters) and fixed rotation relative to turret (in radians)
        // Positive X is forward, Y is left, Z is up from turret rotation center
        // Rotation is the camera's POV angle when the turret is at 0 degrees
        public static Transform3d LIMELIGHT_TRANSFORM_FROM_TURRET_CENTER = new Transform3d(
                        new Translation3d(0.016, -0.138, .105),
                        new Rotation3d(0, 0.523599, 0));
}
