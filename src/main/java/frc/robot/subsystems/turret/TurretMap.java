package frc.robot.subsystems.turret;

import java.util.Map;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TurretMap {
        public static final double TURRET_GEAR_RATIO = 10;
        public static final int TURRET_ROTATOR_CANID = 0;
        public static final boolean TURRET_ROTATOR_INVERTED = false;
        public static final ExtendedPIDConstants TURRET_ROTATOR_PID = new ExtendedPIDConstants();
        public static final double YAW_MAX_MANUAL_SPEED = 1;
        public static final double PITCH_MAX_MANUAL_SPEED = 1;

        public static final int FLYWHEEL_LEFT_CANID = 0;
        public static final int FLYWHEEL_RIGHT_CANID = 0;
        public static final boolean FLYWHEEL_LEFT_INVERTED = false;
        public static final double FLYWHEEL_RAMP_PERIOD = 1;
        public static final ExtendedPIDConstants FLYWHEEL_PID = new ExtendedPIDConstants();
        public static final double FLYWHEEL_IDLE_VELOCITY_RPS = 5.0;
        public static final double FLYWHEEL_RADIUS = 0.0505;
        public static final double FLYWHEEL_MAX_SPEED = 5;
        public static final double FLYWHEEL_MIN_SPEED = 0.0;
        //        public static final double HOOD_MAX_ANGLE_DEGREES = 60.0; // Hood fully retracted
        //        public static final double HOOD_MIN_ANGLE_DEGREES = 20.0; // Hood fully extended
        public static final double HOOD_MAX_ANGLE_DEGREES = 35.1; // Hood fully retracted
        //        public static final double HOOD_MAX_ANGLE_DEGREES = 12.6; // Hood fully retracted
        public static final double HOOD_MIN_ANGLE_DEGREES = 12.6; // Hood fully extended

        public static final int FEEDER_CANID = 0;
        public static final boolean FEEDER_INVERTED = false;
        public static final double FEEDER_VELOCITY_CONVERSION_FACTOR = 1.0;

        // Polar coordinates for easy vector math
        public static final double TURRET_DISTANCE_FROM_ROBOT_CENTER = 0;
        public static final Rotation2d TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT = new Rotation2d()
                        .rotateBy(Rotation2d.kCCW_90deg);
        public static final double TURRET_HEIGHT_ABOVE_GROUND = 0.45;

        public static final Pose3d HUB_GOAL_POSITION = new Pose3d();
        public static final double HUB_OVERSHOOT_HEIGHT = 0.2;
        public static final double MIN_SHOT_DISTANCE_METERS = 0;

        public static final boolean AUTO_MOTION_COMPENSATION = false;
        public static final double AUTO_AIM_YAW_TRIM_DEGREES = 10;

        public static final boolean USE_SPEED_INTERPOLATION = false;
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

        // Limelight offset from turret rotation center (in meters)
        // Positive X is forward, Y is left, Z is up from turret rotation center
        public static double LIMELIGHT_OFFSET_X = 0.0; // Distance forward from turret center
        public static double LIMELIGHT_OFFSET_Y = 0.0; // Distance right from turret center (negative for left)
        public static double LIMELIGHT_OFFSET_Z = 0.0; // Distance above turret center

        // Limelight fixed rotation relative to turret (in radians)
        // This is the camera's POV angle when the turret is at 0 degrees
        public static double LIMELIGHT_PITCH = 0.0; // Vertical tilt
        public static double LIMELIGHT_YAW = 0.0; // Horizontal rotation (should typically be 0)
        public static double LIMELIGHT_ROLL = 0.0; // Camera roll

        // Turret rotation origin offset from robot center (in meters, XYZ from robot center on ground)
        public static double TURRET_CENTER_OFFSET_X = 0.0; // Distance forward from robot center
        public static double TURRET_CENTER_OFFSET_Y = 0.0; // Distance right from robot center (negative for left)
        public static double TURRET_CENTER_OFFSET_Z = 0.0; // Height above ground
}
