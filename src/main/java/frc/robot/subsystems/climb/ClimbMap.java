package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class ClimbMap {
    public static final int CLIMB_MOTOR_CANID = 21;
    public static final double CLIMB_MOTOR_GEAR_RATIO = 225; // Current gear ratio in CAD, subject to change
    public static final boolean CLIMB_MOTOR_INVERTED = false;
    public static final double CLIMB_MOTOR_RAMP_PERIOD = 1;

    // TODO: Implement pressure sensor reading
    public static final int PRESSURE_SENSOR_CHANNEL = 0; // Not a placeholder, 0 is the correct channel

    public static final int FrictionBrakeForwardChannel = 8;
    public static final int FrictionBrakeReverseChannel = 9;

    public static final int SUPPORT_FORWARD_CHANNEL = 6;
    public static final int SUPPORT_REVERSE_CHANNEL = 7;

    public static final int LIMIT_SWITCH_CHANNEL = 1; // Placeholder
    public static final double MAX_CLIMB_MOTOR_PERCENT_OUT = 0.25;

    // TODO: Placeholder, set to actual max extension
    public static final Distance MAX_CLIMB_EXTENSION = Distance.ofBaseUnits(1, Meters);
    public static final Distance CLIMB_AT_SETPOINT_ERROR = Distance.ofBaseUnits(1, Inches); // TODO: Tune
    public static final Distance CLIMB_PULLEY_RADIUS = Distance.ofBaseUnits(1, Inches); // TODO: Measure
}
