package frc.robot.subsystems.climb;

import org.prime.control.ExtendedPIDConstants;

public class ClimbMap {
    public static final int CLIMB_MOTOR_CANID = 0;
    public static final double CLIMB_MOTOR_GEAR_RATIO = 225; // Current gear ratio in CAD, subject to change
    public static final boolean CLIMB_MOTOR_INVERTED = false;
    public static final ExtendedPIDConstants CLIMB_MOTOR_PID = new ExtendedPIDConstants();
    public static final double CLIMB_MOTOR_RAMP_PERIOD = 1;

    public static final int SUPPORT_MOTOR_CANID = 0;
    public static final ExtendedPIDConstants SUPPORT_MOTOR_PID = new ExtendedPIDConstants();
}
