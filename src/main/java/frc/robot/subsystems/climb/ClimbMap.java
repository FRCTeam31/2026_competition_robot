package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Translation3d;
import org.prime.control.ExtendedPIDConstants;

public class ClimbMap {
    public static final int CLIMB_MOTOR_CANID = 0;
    public static final double CLIMB_MOTOR_GEAR_RATIO = 225; // Current gear ratio in CAD, subject to change
    public static final boolean CLIMB_MOTOR_INVERTED = false;
    public static final ExtendedPIDConstants CLIMB_MOTOR_PID = new ExtendedPIDConstants();
    public static final double CLIMB_MOTOR_RAMP_PERIOD = 1;
    public static final double CLIMB_PULLEY_RADIUS = 0.01905;

    public static final int SUPPORT_MOTOR_CANID = 0;
    public static final ExtendedPIDConstants SUPPORT_MOTOR_PID = new ExtendedPIDConstants();

    // TODO: Set channels
    public static final int FrictionBrakeForwardChannel = 0;
    public static final int FrictionBrakeReverseChannel = 0;

    public static final int SUPPORT_FORWARD_CHANNEL = 0;
    public static final int SUPPORT_REVERSE_CHANNEL = 0;

    public static final int UPPER_LIMIT_SWITCH_CHANNEL = 0;
    public static final int LOWER_LIMIT_SWITCH_CHANNEL = 0;

    // Mechanism Root Locations
    public static final Translation3d CLIMB_ROOT_POSITION = new Translation3d(-0.321, 0.083, 0.134);
    public static final Translation3d SUPPORT_ROOT_POSITION = new Translation3d(-0.354, 0.141, 0.393);
}
