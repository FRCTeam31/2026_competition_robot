package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/**
 * Constants for the Climb subsystem including CAN IDs, gear ratios,
 * pneumatic channels, and physical measurements.
 */
public class ClimbMap {

    // Motor
    public static final int CLIMB_MOTOR_CANID = 21;
    public static final double CLIMB_MOTOR_GEAR_RATIO = 225; // Current gear ratio in CAD, subject to change
    public static final boolean CLIMB_MOTOR_INVERTED = false;
    public static final double CLIMB_MOTOR_RAMP_PERIOD = 1;
    public static final double MAX_CLIMB_MOTOR_PERCENT_OUT = 0.25;

    // Friction Brake Solenoid
    public static final int FRICTION_BRAKE_FORWARD_CHANNEL = 8;
    public static final int FRICTION_BRAKE_REVERSE_CHANNEL = 9;

    // Support Solenoid
    public static final int SUPPORT_FORWARD_CHANNEL = 6;
    public static final int SUPPORT_REVERSE_CHANNEL = 7;

    // Limit Switch
    public static final int LIMIT_SWITCH_CHANNEL = 1; // Placeholder

    // Physical Measurements
    public static final Distance MAX_CLIMB_EXTENSION = Distance.ofBaseUnits(1, Meters); // TODO: Placeholder, set to actual max extension
    public static final Distance CLIMB_AT_SETPOINT_ERROR = Distance.ofBaseUnits(1, Inches); // TODO: Tune
    public static final Distance CLIMB_PULLEY_RADIUS = Distance.ofBaseUnits(1, Inches); // TODO: Measure
}
