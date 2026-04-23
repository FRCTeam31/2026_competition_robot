package frc.robot.subsystems.leds;

import org.prime.util.LEDPatternUtil;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDPatterns {

        // ------------------------ Timing Constants --------------------------
        private static final Time SLOW_BLINK = Units.Milliseconds.of(500);
        private static final Time FAST_BLINK = Units.Milliseconds.of(100);
        private static final Time QUICK_FLASH_ON = Units.Milliseconds.of(50);
        private static final Time QUICK_FLASH_OFF = Units.Milliseconds.of(100);
        private static final Time SLOW_ALTERNATE = Units.Milliseconds.of(500);
        private static final Time FAST_ALTERNATE = Units.Milliseconds.of(100);

        // ------------------------ Red --------------------------------
        public static final LEDPattern RedSlowBlink = slowBlink(Color.kRed);
        public static final LEDPattern RedFastBlink = fastBlink(Color.kRed);
        public static final LEDPattern RedQuickFlash = quickFlash(Color.kRed);
        public static final LEDPattern RedSlowBurstBlink = slowBurstBlink(Color.kRed);
        public static final LEDPattern RedFastBurstBlink = fastBurstBlink(Color.kRed);
        public static final LEDPattern RedAlternatingSlow = alternatingSlow(Color.kRed);
        public static final LEDPattern RedAlternatingFast = alternatingFast(Color.kRed);
        public static final LEDPattern RedTwoToneSlow = twoToneSlow(Color.kRed);
        public static final LEDPattern RedTwoToneFast = twoToneFast(Color.kRed);

        // ------------------------ Yellow -----------------------------
        public static final LEDPattern YellowSlowBlink = slowBlink(Color.kYellow);
        public static final LEDPattern YellowFastBlink = fastBlink(Color.kYellow);
        public static final LEDPattern YellowQuickFlash = quickFlash(Color.kYellow);
        public static final LEDPattern YellowSlowBurstBlink = slowBurstBlink(Color.kYellow);
        public static final LEDPattern YellowFastBurstBlink = fastBurstBlink(Color.kYellow);
        public static final LEDPattern YellowAlternatingSlow = alternatingSlow(Color.kYellow);
        public static final LEDPattern YellowAlternatingFast = alternatingFast(Color.kYellow);
        public static final LEDPattern YellowTwoToneSlow = twoToneSlow(Color.kYellow);
        public static final LEDPattern YellowTwoToneFast = twoToneFast(Color.kYellow);

        // ------------------------ Blue -------------------------------
        public static final LEDPattern BlueSlowBlink = slowBlink(Color.kBlue);
        public static final LEDPattern BlueFastBlink = fastBlink(Color.kBlue);
        public static final LEDPattern BlueQuickFlash = quickFlash(Color.kBlue);
        public static final LEDPattern BlueSlowBurstBlink = slowBurstBlink(Color.kBlue);
        public static final LEDPattern BlueFastBurstBlink = fastBurstBlink(Color.kBlue);
        public static final LEDPattern BlueAlternatingSlow = alternatingSlow(Color.kBlue);
        public static final LEDPattern BlueAlternatingFast = alternatingFast(Color.kBlue);
        public static final LEDPattern BlueTwoToneSlow = twoToneSlow(Color.kBlue);
        public static final LEDPattern BlueTwoToneFast = twoToneFast(Color.kBlue);

        // ------------------------ Green ---------------------
        public static final LEDPattern GreenSlowBlink = slowBlink(Color.kGreen);
        public static final LEDPattern GreenFastBlink = fastBlink(Color.kGreen);
        public static final LEDPattern GreenQuickFlash = quickFlash(Color.kGreen);
        public static final LEDPattern GreenSlowBurstBlink = slowBurstBlink(Color.kGreen);
        public static final LEDPattern GreenFastBurstBlink = fastBurstBlink(Color.kGreen);
        public static final LEDPattern GreenAlternatingSlow = alternatingSlow(Color.kGreen);
        public static final LEDPattern GreenAlternatingFast = alternatingFast(Color.kGreen);
        public static final LEDPattern GreenTwoToneSlow = twoToneSlow(Color.kGreen);
        public static final LEDPattern GreenTwoToneFast = twoToneFast(Color.kGreen);

        // -------------------- Alternating Blink Patterns ----------------------
        public static final LEDPattern GreenRedAlternating = LEDPatternUtil.alternating(
                        GreenFastBlink,
                        RedFastBlink,
                        SLOW_ALTERNATE);
        public static final LEDPattern BlueYellowAlternating = LEDPatternUtil.alternating(
                        BlueFastBlink,
                        YellowFastBlink,
                        SLOW_ALTERNATE);
        public static final LEDPattern BlueGreenAlternatingBurstBlink = LEDPatternUtil.split(
                        BlueSlowBurstBlink,
                        GreenSlowBurstBlink,
                        LEDMap.PixelsPerSection / 2);

        // ==================== Factory Methods ================================

        /** Creates a slow-blinking pattern (500ms on, 500ms off) of the given color. */
        public static LEDPattern slowBlink(Color color) {
                return LEDPatternUtil.blink(color, SLOW_BLINK, SLOW_BLINK);
        }

        /** Creates a fast-blinking pattern (100ms on, 100ms off) of the given color. */
        public static LEDPattern fastBlink(Color color) {
                return LEDPatternUtil.blink(color, FAST_BLINK, FAST_BLINK);
        }

        /** Creates a fast-blinking pattern (100ms on, 100ms off) of the given color. */
        public static LEDPattern fastBlink(Color color, Time blinkTime) {
                return LEDPatternUtil.blink(color, blinkTime, blinkTime);
        }

        /** Creates a quick-flash pattern (50ms on, 100ms off) of the given color. */
        public static LEDPattern quickFlash(Color color) {
                return LEDPatternUtil.blink(color, QUICK_FLASH_ON, QUICK_FLASH_OFF);
        }

        /**
         * Creates a slow burst-blink pattern: a fast-blinking base shown for 500ms,
         * then off for 500ms, repeating.
         */
        public static LEDPattern slowBurstBlink(Color color) {
                return LEDPatternUtil.burstBlink(
                                fastBlink(color),
                                SLOW_BLINK,
                                SLOW_BLINK);
        }

        /**
         * Creates a fast burst-blink pattern: a fast-blinking base shown for 100ms,
         * then off for 100ms, repeating.
         */
        public static LEDPattern fastBurstBlink(Color color) {
                return LEDPatternUtil.burstBlink(
                                fastBlink(color),
                                FAST_BLINK,
                                FAST_BLINK);
        }

        /**
         * Creates a slow alternating pattern: one half shows the color while the
         * other is off, then they swap every 500ms.
         */
        public static LEDPattern alternatingSlow(Color color) {
                return LEDPatternUtil.alternating(
                                LEDPattern.solid(color),
                                LEDPattern.kOff,
                                SLOW_ALTERNATE);
        }

        /**
         * Creates a fast alternating pattern: one half shows the color while the
         * other is off, then they swap every 100ms.
         */
        public static LEDPattern alternatingFast(Color color) {
                return LEDPatternUtil.alternating(
                                LEDPattern.solid(color),
                                LEDPattern.kOff,
                                FAST_ALTERNATE);
        }

        /**
         * Creates a slow two-tone pattern: the given color alternates with black
         * across two halves of the strip, swapping every 500ms.
         */
        public static LEDPattern twoToneSlow(Color color) {
                return LEDPatternUtil.twoTone(color, Color.kBlack, SLOW_ALTERNATE);
        }

        /**
         * Creates a fast two-tone pattern: the given color alternates with black
         * across two halves of the strip, swapping every 100ms.
         */
        public static LEDPattern twoToneFast(Color color) {
                return LEDPatternUtil.twoTone(color, Color.kBlack, FAST_ALTERNATE);
        }

        /**
         * Creates a two-color alternating pattern: left half shows one color's
         * fast blink, right half shows the other, swapping sides every 500ms.
         */
        public static LEDPattern twoColorAlternating(Color left, Color right) {
                return LEDPatternUtil.alternating(
                                fastBlink(left),
                                fastBlink(right),
                                SLOW_ALTERNATE);
        }
}
