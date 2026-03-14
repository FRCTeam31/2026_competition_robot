package frc.robot.subsystems.turret;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
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
import org.prime.util.IDWController;

public class TurretMap {
        // ──────────────────────── Feature Flags ─────────────────────────
        public static final boolean UPDATE_LIMELIGHT_POSE = false;
        public static final boolean AUTO_MOTION_COMPENSATION = false;
        public static final boolean USE_SPEED_INTERPOLATION = false;
        public static final boolean YAW_DEADZONE_ENABLED = true;
        public static final boolean USE_LIMELIGHT_TARGETING = false;
        public static final boolean USE_LIMELIGHT_YAW_CORRECTION = false;

        // ──────────────────────── Turret Yaw ────────────────────────────
        public static final int TURRET_ROTATOR_CANID = 17;
        public static final boolean TURRET_ROTATOR_INVERTED = false;
        public static final double TURRET_GEAR_RATIO = 10;
        public static final ExtendedPIDConstants TURRET_ROTATOR_PID = new ExtendedPIDConstants(15, 0, 0);
        // public static final double YAW_MOTION_MAGIC_CRUISE_VELOCITY = 1000; // sensor units per 100ms (~88°/s)
        // public static final double YAW_MOTION_MAGIC_ACCELERATION = 2000; // sensor units per 100ms per second (~0.5s to cruise)
        public static final double YAW_MAX_MANUAL_PERCENT_OUT = 1;
        public static final double AUTO_AIM_YAW_TRIM_DEGREES = 10;
        public static final double TURRET_CORRECTION_THRESHOLD_DEGREES = 2.0;
        public static final double YAW_ON_TARGET_TOLERANCE_DEGREES = 2.0;
        public static final int TURRET_YAW_ENCODER_TICKS_PER_TURRET_DEGREE = 4096 / 360; // TODO: Change to actual value later
        public static final Angle YAW_RESET_ANGLE = Degrees.of(180);
        public static final int TURRET_RESET_SWITCH_CHANNEL = 0;

        // ──────────────────────── Turret Dead Zone ──────────────────────
        // The arc from DEADZONE_START to DEADZONE_END (going clockwise
        // through the larger angle) that the turret physically cannot traverse.
        // Both values are in degrees, measured in the same frame as turret rotation (0–360).
        // Example: START = 170, END = 190 means a 20° dead zone centered on 180°.
        public static final double DEADZONE_START_DEGREES = 315;
        public static final double DEADZONE_END_DEGREES = 60;

        // ──────────────────────── Turret Geometry ───────────────────────
        public static final double TURRET_DISTANCE_FROM_ROBOT_CENTER = 0;
        public static final Rotation2d TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT = new Rotation2d()
                        .rotateBy(Rotation2d.kCCW_90deg);
        public static final double TURRET_HEIGHT_ABOVE_GROUND = 0.45;
        public static Translation3d TURRET_ROBOT_ORIGIN = new Translation3d(
                        Units.inchesToMeters(8.25),
                        Units.inchesToMeters(5.75),
                        Units.inchesToMeters(15.894));

        // ──────────────────────── Flywheel ──────────────────────────────
        public static final int FLYWHEEL_LEFT_CANID = 19;
        public static final int FLYWHEEL_RIGHT_CANID = 20;
        public static final boolean FLYWHEEL_LEFT_INVERTED = false;
        public static final double FLYWHEEL_RAMP_PERIOD = 1;
        public static final ExtendedPIDConstants FLYWHEEL_PID = new ExtendedPIDConstants(
                        0,
                        0,
                        0,
                        0,
                        0.1645,
                        0.013876,
                        0.075161);
        public static final AngularVelocity FLYWHEEL_IDLE_VELOCITY = RotationsPerSecond.of(1);
        public static final double FLYWHEEL_RADIUS = 0.0505;
        public static final double FLYWHEEL_MAX_SPEED_RPS = 90;
        public static final double FLYWHEEL_MIN_SPEED_RPS = 0.0;
        public static final double FLYWHEEL_AT_SPEED_TOLERANCE_PERCENT = 5.0; // 5% tolerance
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_SPEED_MAP = InterpolatingDoubleTreeMap
                        .ofEntries(
                                        Map.entry(2.0, 300.0),
                                        Map.entry(3.0, 400.0),
                                        Map.entry(4.0, 500.0),
                                        Map.entry(5.0, 600.0),
                                        Map.entry(6.0, 700.0),
                                        Map.entry(7.0, 800.0),
                                        Map.entry(8.0, 900.0),
                                        Map.entry(9.0, 1000.0));
        public static final List<IDWController.Entry> FLYWHEEL_IDW_ENTRIES = List.of(
                        new IDWController.Entry(0, 0, 0), // Example implementation, will replace with real data
                        new IDWController.Entry(1, 1, 1) // Target Velocity (in m/s), Hood Angle (in degrees), Flywheel Velocity (in Rotations Per Second)
        );

        // ──────────────────────── Hood ───────────────────────────────────
        public static final int HOOD_CAN_ID = 18;
        public static final boolean HOOD_INVERTED = false;
        public static final double HOOD_MAX_ANGLE_DEGREES = 35.1; // Hood fully retracted
        // public static final double HOOD_MIN_ANGLE_DEGREES = 12.6; // Hood fully extended
        public static final double HOOD_MIN_ANGLE_DEGREES = HOOD_MAX_ANGLE_DEGREES; // Hood fully extended
        public static final ExtendedPIDConstants HOOD_PID = new ExtendedPIDConstants(0.1, 0, 0);
        public static final double HOOD_MAX_MOTION_MAX_VELOCITY = 100; // RPM // TODO: Tune
        public static final double HOOD_MAX_MOTION_MAX_ACCELERATION = 200; // RPM per second // TODO: Tune
        public static final double HOOD_MAX_MOTION_ALLOWED_ERROR = 0.5; // degrees // TODO: Tune
        public static final double PITCH_MAX_MANUAL_PERCENT_OUT = 0.2; // TODO: Tune
        public static final double HOOD_ON_TARGET_TOLERANCE_DEGREES = 1.0;
        public static final Distance HOOD_GEAR_RADIUS = Millimeters.of(10);
        public static final AngularVelocity HOOD_SIM_MAX_SPEED = RadiansPerSecond.of(183.33 * Math.PI * 2);
        public static final double HOOD_GEAR_RATIO = 6.333;
        public static final boolean HOOD_ENCODER_INVERTED = false;

        // ──────────────────────── Feeder ─────────────────────────────────
        public static final int FEEDER_CANID = 16;
        public static final boolean FEEDER_INVERTED = false;
        public static final double MAX_FEED_PERCENT_OUT = 0.4;
        public static final double FEEDER_VELOCITY_CONVERSION_FACTOR = 1.0;

        // ──────────────────────── Manual Mode Steps ──────────────────────
        /** Amount to adjust the flywheel speed per button press in MANUAL mode (RPS) */
        public static final double MANUAL_FLYWHEEL_STEP_RPS = 2.0;
        /** Amount to adjust the hood angle per button press in MANUAL mode (degrees) */
        public static final double MANUAL_HOOD_STEP_DEGREES = 1.0;
        /** Amount to adjust the turret yaw per button press in MANUAL mode (degrees) */
        public static final double MANUAL_YAW_STEP_DEGREES = 10.0;

        // ──────────────────────── Home Positions ─────────────────────────
        /** Yaw home position in degrees (180° = straight back) */
        public static final double YAW_HOME_DEGREES = 180.0;
        /** Hood home angle in degrees (fully up / hood lowered) */
        public static final double HOOD_HOME_DEGREES = HOOD_MAX_ANGLE_DEGREES;

        // ──────────────────────── Shot Targeting ─────────────────────────
        public static final Pose3d HUB_GOAL_POSITION = new Pose3d();
        public static final double HUB_OVERSHOOT_HEIGHT = 0.2;
        public static final double MIN_SHOT_DISTANCE_METERS = 0;
        public static final double FLYWHEEL_GEAR_RATIO = 32d / 50d; // 32t on motor gear, 50t on flywheel gear

        // ──────────────────────── Limelight ──────────────────────────────
        // Offset from turret rotation center (in meters) and fixed rotation relative to turret (in radians)
        // Positive X is forward, Y is left, Z is up from turret rotation center
        // Rotation is the camera's POV angle when the turret is at 0 degrees
        public static Transform3d LIMELIGHT_TRANSFORM_FROM_TURRET_CENTER = new Transform3d(
                        new Translation3d(0.016, -0.138, .105),
                        new Rotation3d(0, 0.523599, 0));
}
