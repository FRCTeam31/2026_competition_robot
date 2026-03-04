package frc.robot.subsystems.leds;

public class LEDMap {
    public static final int PwmPort = 0;
    public static final int PixelsPerSection = 8;
    public static final int SectionCount = 1;
    public static final int TotalPixels = PixelsPerSection * SectionCount;
    public static final int MaxLoopErrorsBeforeShutdown = 3;
}
