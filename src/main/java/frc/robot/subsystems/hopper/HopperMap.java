package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Translation3d;
import org.prime.util.MutVector;

public class HopperMap {
    public static final int HopperForwardChannel = 0;
    public static final int HopperReverseChannel = 0;
    public static final int IntakeForwardChannel = 0;
    public static final int IntakeReverseChannel = 0;

    public static final int CANID = 0; // placeholder can value
    public static final double HOPPER_FEED_GEAR_RATIO = 27;

    public static final double HopperPulseDelay = 0;

    // Mechanism Root Locations
    public static final Translation3d HOPPER_ROOT_POSITION = new Translation3d(0.333, 0, 0.390);
    public static final Translation3d INTAKE_ROOT_POSITION = new Translation3d(0.329, 0, 0.264);
    public static final Translation3d INTAKE_FEED_ROOT_POSITION = new Translation3d(0.295, 0, -0.117);
    public static final Translation3d TOP_FEED_BAR_ROOT_POSITION = new Translation3d(0.016, 0, 0.341);
    public static final Translation3d BOTTOM_FEED_BAR_ROOT_POSITION = new Translation3d(0.009, 0, 0.155);
}
