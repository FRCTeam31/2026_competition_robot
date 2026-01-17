package frc.robot.subsystems.turret;

import org.prime.control.ExtendedPIDConstants;

public class TurretMap {
    public static final double TURRET_GEAR_RATIO = 10;
    public static final int TURRET_ROTATOR_CANID = 0;
    public static final boolean TURRET_ROTATOR_INVERTED = false;
    public static final ExtendedPIDConstants TURRET_ROTATOR_PID = new ExtendedPIDConstants();

    public static final int FLYWHEEL_LEFT_CANID = 0;
    public static final int FLYWHEEL_RIGHT_CANID = 0;
    public static final boolean FLYWHEEL_LEFT_INVERTED = false;
    public static final double FLYWHEEL_RAMP_PERIOD = 1;
}
