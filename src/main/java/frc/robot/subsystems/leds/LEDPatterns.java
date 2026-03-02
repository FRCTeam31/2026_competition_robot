package frc.robot.subsystems.leds;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDPatterns {

        // ------------------------ Timing Constants --------------------------
        private static final Time SLOW_BLINK_ON = Units.Milliseconds.of(500);
        private static final Time SLOW_BLINK_OFF = Units.Milliseconds.of(750);
        private static final Time FAST_BLINK_ON = Units.Milliseconds.of(100);
        private static final Time FAST_BLINK_OFF = Units.Milliseconds.of(200);
        private static final Time QUICK_FLASH_ON = Units.Milliseconds.of(50);
        private static final Time QUICK_FLASH_OFF = Units.Milliseconds.of(100);
        private static final Time SLOW_ALTERNATE = Units.Milliseconds.of(500);
        private static final Time FAST_ALTERNATE = Units.Milliseconds.of(150);

        // ------------------------ Blink: Red --------------------------------
        public static final LEDPattern RedSlowBlink = Blink(Color.kRed, SLOW_BLINK_ON, SLOW_BLINK_OFF);
        public static final LEDPattern RedFastBlink = Blink(Color.kRed, FAST_BLINK_ON, FAST_BLINK_OFF);
        public static final LEDPattern RedQuickFlash = Blink(Color.kRed, QUICK_FLASH_ON, QUICK_FLASH_OFF);

        // ------------------------ Blink: Yellow -----------------------------
        public static final LEDPattern YellowSlowBlink = Blink(Color.kYellow, SLOW_BLINK_ON, SLOW_BLINK_OFF);
        public static final LEDPattern YellowFastBlink = Blink(Color.kYellow, FAST_BLINK_ON, FAST_BLINK_OFF);

        // ------------------------ Blink: Blue -------------------------------
        public static final LEDPattern BlueSlowBlink = Blink(Color.kBlue, SLOW_BLINK_ON, SLOW_BLINK_OFF);
        public static final LEDPattern BlueFastBlink = Blink(Color.kBlue, FAST_BLINK_ON, FAST_BLINK_OFF);

        // ------------------------ Two-Tone: Green/Black ---------------------
        // 4 LEDs green, 4 LEDs black, alternating back and forth
        public static final LEDPattern GreenTwoToneSlow = TwoTone(Color.kGreen, Color.kBlack, SLOW_ALTERNATE);
        public static final LEDPattern GreenTwoToneFast = TwoTone(Color.kGreen, Color.kBlack, FAST_ALTERNATE);

        // ------------------------ Two-Tone: Yellow/Black --------------------
        public static final LEDPattern YellowTwoToneSlow = TwoTone(Color.kYellow, Color.kBlack, SLOW_ALTERNATE);
        public static final LEDPattern YellowTwoToneFast = TwoTone(Color.kYellow, Color.kBlack, FAST_ALTERNATE);

        // ------------------------ Two-Tone: Blue/Black ----------------------
        public static final LEDPattern BlueTwoToneSlow = TwoTone(Color.kBlue, Color.kBlack, SLOW_ALTERNATE);
        public static final LEDPattern BlueTwoToneFast = TwoTone(Color.kBlue, Color.kBlack, FAST_ALTERNATE);

        // ------------------------ Utility Methods ---------------------------
        public static LEDPattern Blink(Color color, Time onPeriod, Time offPeriod) {
                return LEDPattern.solid(color).blink(onPeriod, offPeriod);
        }

        /**
         * Creates a two-tone alternating pattern: the first half of the strip shows
         * one color and the second half shows another. The two halves blink in
         * opposite phase so they visually swap back and forth at the given period.
         *
         * @param primary   the first color (LEDs 0-3 in phase A)
         * @param secondary the second color (LEDs 4-7 in phase A)
         * @param period    how long each phase lasts before swapping
         */
        public static LEDPattern TwoTone(Color primary, Color secondary, Time period) {
                var step1Mask = LEDPattern.steps(GetDualColorMask(Color.kWhite, Color.kBlack, 0.5))
                                .blink(period);
                var step2Mask = LEDPattern.steps(GetDualColorMask(Color.kBlack, Color.kWhite, 0.5))
                                .blink(period);

                var base = LEDPattern.steps(GetDualColorMask(primary, secondary, 0.5));
                var mask = step1Mask.overlayOn(step2Mask);

                return base.mask(mask);
        }

        public static Map<Number, Color> GetSingleColorMask(Color color, Number position) {
                return GetDualColorMask(color, Color.kBlack, position);
        }

        public static Map<Number, Color> GetDualColorMask(Color color1, Color color2, Number position) {
                return Map.of(0, color1, position, color2);
        }
}
