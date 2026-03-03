package frc.robot.subsystems.climb;

import org.prime.control.ExtendedPIDConstants;

/**
 * Constants for the Climb subsystem including CAN IDs, gear ratios,
 * pneumatic channels, and MAXMotion parameters.
 */
public class ClimbMap {

    // Motor
    public static final int CLIMB_MOTOR_CANID = 21;
    public static final boolean CLIMB_MOTOR_INVERTED = false;

    // MAXMotion position control
    public static final ExtendedPIDConstants CLIMB_PID = new ExtendedPIDConstants(0.1, 0, 0); // TODO: Tune
    public static final double MAX_MOTION_MAX_VELOCITY = 5; // Rotations per second // TODO: Tune
    public static final double MAX_MOTION_MAX_ACCELERATION = 10; // Rotations per second^2 // TODO: Tune
    public static final double MAX_MOTION_ALLOWED_ERROR = 0.1; // Rotations // TODO: Tune

    // Climb setpoints (motor rotations at the output shaft, after gearing)
    public static final double RETRACTED_ROTATIONS = 0; // Home position (limit switch)
    public static final double EXTENDED_ROTATIONS = 50; // TODO: Measure actual full extension in motor rotations

    // Soft limits (motor rotations)
    public static final float FORWARD_SOFT_LIMIT = (float) EXTENDED_ROTATIONS + 1; // Small margin past extension
    public static final float REVERSE_SOFT_LIMIT = -1; // Small margin past home to allow zeroing

    // Friction Brake Solenoid
    public static final int FRICTION_BRAKE_FORWARD_CHANNEL = 8;
    public static final int FRICTION_BRAKE_REVERSE_CHANNEL = 9;

    // Support Solenoid
    public static final int SUPPORT_FORWARD_CHANNEL = 7;
    public static final int SUPPORT_REVERSE_CHANNEL = 6;

    // Limit Switch
    public static final int LIMIT_SWITCH_CHANNEL = 1; // Placeholder
}
