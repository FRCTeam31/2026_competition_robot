package frc.robot.subsystems.turret;

import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretMap {
    public static final double TURRET_GEAR_RATIO = 10;
    public static final int TURRET_ROTATOR_CANID = 0;
    public static final boolean TURRET_ROTATOR_INVERTED = false;
    public static final ExtendedPIDConstants TURRET_ROTATOR_PID = new ExtendedPIDConstants();

    public static final int FLYWHEEL_LEFT_CANID = 0;
    public static final int FLYWHEEL_RIGHT_CANID = 0;
    public static final boolean FLYWHEEL_LEFT_INVERTED = false;
    public static final double FLYWHEEL_RAMP_PERIOD = 1;
    public static final ExtendedPIDConstants FLYWHEEL_PID = new ExtendedPIDConstants();
    public static final double FLYWHEEL_IDLE_VELOCITY_RPS = 1.0;
    public static final double FLYWHEEL_RADIUS = 0;

    public static final int FEEDER_CANID = 0;
    public static final boolean FEEDER_INVERTED = false;
    public static final double FEEDER_VELOCITY_CONVERSION_FACTOR = 1.0;

    public static final double TURRET_DISTANCE_FROM_ROBOT_CENTER = 0;
    public static final Rotation2d TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT = new Rotation2d()
            .rotateBy(Rotation2d.kCCW_90deg);
    public static final double TURRET_HEIGHT_ABOVE_GROUND = 0;

    public static final Pose3d HUB_GOAL_POSITION = new Pose3d();
    public static final double HUB_OVERSHOOT_HEIGHT = 0;

    public static final boolean AUTO_MOTION_COMPENSATION = false;

}
