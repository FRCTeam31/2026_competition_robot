package frc.robot.subsystems.leds;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDPatterns {

    // ──────────────────────── Timing Constants ──────────────────────────
    private static final Time SLOW_BLINK_ON = Units.Milliseconds.of(500);
    private static final Time SLOW_BLINK_OFF = Units.Milliseconds.of(750);
    private static final Time FAST_BLINK_ON = Units.Milliseconds.of(100);
    private static final Time FAST_BLINK_OFF = Units.Milliseconds.of(200);
    private static final Frequency SLOW_SCROLL_SPEED = Units.Hertz.of(0.5);
    private static final Frequency FAST_SCROLL_SPEED = Units.Hertz.of(2.0);

    // ──────────────────────── Blink: Red ────────────────────────────────
    public static final LEDPattern RedSlowBlink = Blink(Color.kRed, SLOW_BLINK_ON, SLOW_BLINK_OFF);
    public static final LEDPattern RedFastBlink = Blink(Color.kRed, FAST_BLINK_ON, FAST_BLINK_OFF);

    // ──────────────────────── Blink: Yellow ─────────────────────────────
    public static final LEDPattern YellowSlowBlink = Blink(Color.kYellow, SLOW_BLINK_ON, SLOW_BLINK_OFF);
    public static final LEDPattern YellowFastBlink = Blink(Color.kYellow, FAST_BLINK_ON, FAST_BLINK_OFF);

    // ──────────────────────── Blink: Blue ───────────────────────────────
    public static final LEDPattern BlueSlowBlink = Blink(Color.kBlue, SLOW_BLINK_ON, SLOW_BLINK_OFF);
    public static final LEDPattern BlueFastBlink = Blink(Color.kBlue, FAST_BLINK_ON, FAST_BLINK_OFF);

    // ──────────────────────── Scroll: Yellow ────────────────────────────
    public static final LEDPattern YellowScrollUpSlow = ScrollUp(GetSingleColorMask(Color.kYellow, 0.5),
            SLOW_SCROLL_SPEED);
    public static final LEDPattern YellowScrollUpFast = ScrollUp(GetSingleColorMask(Color.kYellow, 0.5),
            FAST_SCROLL_SPEED);
    public static final LEDPattern YellowScrollDownSlow = ScrollDown(GetSingleColorMask(Color.kYellow, 0.5),
            SLOW_SCROLL_SPEED);
    public static final LEDPattern YellowScrollDownFast = ScrollDown(GetSingleColorMask(Color.kYellow, 0.5),
            FAST_SCROLL_SPEED);

    // ──────────────────────── Scroll: Green ─────────────────────────────
    public static final LEDPattern GreenScrollUpSlow = ScrollUp(GetSingleColorMask(Color.kGreen, 0.5),
            SLOW_SCROLL_SPEED);
    public static final LEDPattern GreenScrollUpFast = ScrollUp(GetSingleColorMask(Color.kGreen, 0.5),
            FAST_SCROLL_SPEED);
    public static final LEDPattern GreenScrollDownSlow = ScrollDown(GetSingleColorMask(Color.kGreen, 0.5),
            SLOW_SCROLL_SPEED);
    public static final LEDPattern GreenScrollDownFast = ScrollDown(GetSingleColorMask(Color.kGreen, 0.5),
            FAST_SCROLL_SPEED);

    // ──────────────────────── Scroll: Blue ─────────────────────────────
    public static final LEDPattern BlueScrollUpSlow = ScrollUp(GetSingleColorMask(Color.kBlue, 0.5),
            SLOW_SCROLL_SPEED);
    public static final LEDPattern BlueScrollUpFast = ScrollUp(GetSingleColorMask(Color.kBlue, 0.5),
            FAST_SCROLL_SPEED);
    public static final LEDPattern BlueScrollDownSlow = ScrollDown(GetSingleColorMask(Color.kBlue, 0.5),
            SLOW_SCROLL_SPEED);
    public static final LEDPattern BlueScrollDownFast = ScrollDown(GetSingleColorMask(Color.kBlue, 0.5),
            FAST_SCROLL_SPEED);

    // ──────────────────────── Utility Methods ───────────────────────────
    public static LEDPattern Blink(Color color, Time onPeriod, Time offPeriod) {
        return LEDPattern.solid(color).blink(onPeriod, offPeriod);
    }

    public static LEDPattern ScrollUp(Map<Number, Color> colorMap, Frequency scrollFrequency) {
        return LEDPattern.steps(colorMap).scrollAtRelativeSpeed(scrollFrequency);
    }

    public static LEDPattern ScrollDown(Map<Number, Color> colorMap, Frequency scrollFrequency) {
        return ScrollUp(colorMap, scrollFrequency).reversed().scrollAtRelativeSpeed(scrollFrequency).reversed();
    }

    public static Map<Number, Color> GetSingleColorMask(Color color, Number position) {
        return GetDualColorMask(color, Color.kBlack, position);
    }

    public static Map<Number, Color> GetDualColorMask(Color color1, Color color2, Number position) {
        return Map.of(0, color1, position, color2);
    }
}
