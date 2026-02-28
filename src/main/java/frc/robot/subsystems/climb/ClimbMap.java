package frc.robot.subsystems.climb;

import org.prime.control.ExtendedPIDConstants;

public class ClimbMap {
    public static final int CLIMB_MOTOR_CANID = 21;
    public static final double CLIMB_MOTOR_GEAR_RATIO = 225; // Current gear ratio in CAD, subject to change
    public static final boolean CLIMB_MOTOR_INVERTED = false;
    public static final ExtendedPIDConstants CLIMB_MOTOR_PID = new ExtendedPIDConstants();
    public static final double CLIMB_MOTOR_RAMP_PERIOD = 1;

    public static final ExtendedPIDConstants SUPPORT_MOTOR_PID = new ExtendedPIDConstants();

    public static final int FrictionBrakeForwardChannel = 8;
    public static final int FrictionBrakeReverseChannel = 9;

    public static final int SUPPORT_FORWARD_CHANNEL = 6;
    public static final int SUPPORT_REVERSE_CHANNEL = 7;

    public static final int UPPER_LIMIT_SWITCH_CHANNEL = 0; // Placeholder
    public static final int LOWER_LIMIT_SWITCH_CHANNEL = 1; // Placeholder
}
