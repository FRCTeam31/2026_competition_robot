package frc.robot.subsystems.prism;

import org.prime.prism.Prism.ColorOrder;
import org.prime.prism.Prism.StripConfig;

public class PrismMap {
    public static final int STRIP_COUNT = 4;
    public static final int PIXELS_PER_STRIP = 8;
    public static final int UPDATE_RATE_MS = 8; // ~125 fps
    public static final int MAX_LOOP_ERRORS_BEFORE_SHUTDOWN = 3;

    /** Default color order for all strips */
    public static final ColorOrder DEFAULT_COLOR_ORDER = ColorOrder.GRB;

    /** COM port name for simulation (PC serial port to real Prism device) */
    public static final String SIM_COM_PORT = "COM12";
    public static final int SIM_BAUD_RATE = 115200; // Must match firmware Serial.begin() — rate is nominal for USB CDC

    /** PWM port used for the WPILib AddressableLED mirror in simulation */
    public static final int SIM_LED_PWM_PORT = 0;

    /** Default strip configurations */
    public static final StripConfig[] STRIP_CONFIGS = new StripConfig[] {
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
            new StripConfig(PIXELS_PER_STRIP, DEFAULT_COLOR_ORDER),
    };
}
