package org.prime.util;

import java.util.Map;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Microseconds;

/**
 * Utility methods for creating custom {@link LEDPattern} compositions
 * that go beyond the built-in WPILib combinators.
 */
public final class LEDPatternUtil {

    private LEDPatternUtil() {
        // Prevent instantiation
    }

    // =========================== Pattern Factories ============================

    /**
     * Creates a simple blinking pattern of a single solid color.
     *
     * @param color    The color to blink
     * @param onPeriod Duration the color is visible
     * @param offPeriod Duration the LEDs are off
     * @return A blinking {@link LEDPattern}
     */
    public static LEDPattern blink(Color color, Time onPeriod, Time offPeriod) {
        long onMicros = (long) onPeriod.in(Microseconds);
        long totalCycleMicros = onMicros + (long) offPeriod.in(Microseconds);
        var solid = LEDPattern.solid(color);
        long[] startTime = { -1 };

        return (reader, writer) -> {
            long now = RobotController.getTime();
            if (startTime[0] < 0) {
                startTime[0] = now;
            }
            long t = (now - startTime[0]) % totalCycleMicros;
            if (t < onMicros) {
                solid.applyTo(reader, writer);
            } else {
                LEDPattern.kOff.applyTo(reader, writer);
            }
        };
    }

    /**
     * Creates a blinking pattern of a single solid color with a one-time startup
     * delay. The LEDs stay off for the specified delay duration after the pattern
     * first starts, then begin blinking normally. The delay only applies once —
     * subsequent cycles run without pause.
     *
     * @param color     The color to blink
     * @param onPeriod  Duration the color is visible
     * @param offPeriod Duration the LEDs are off
     * @param delay     One-time delay before the blink cycle begins
     * @return A delayed blinking {@link LEDPattern}
     */
    public static LEDPattern blink(Color color, Time onPeriod, Time offPeriod, Time delay) {
        long onMicros = (long) onPeriod.in(Microseconds);
        long totalCycleMicros = onMicros + (long) offPeriod.in(Microseconds);
        long delayMicros = (long) delay.in(Microseconds);
        var solid = LEDPattern.solid(color);
        long[] startTime = { -1 };

        return (reader, writer) -> {
            long now = RobotController.getTime();
            if (startTime[0] < 0) {
                startTime[0] = now;
            }
            long elapsed = now - startTime[0];
            if (elapsed < delayMicros) {
                LEDPattern.kOff.applyTo(reader, writer);
                return;
            }
            long t = (elapsed - delayMicros) % totalCycleMicros;
            if (t < onMicros) {
                solid.applyTo(reader, writer);
            } else {
                LEDPattern.kOff.applyTo(reader, writer);
            }
        };
    }

    /**
     * Creates a pattern that shows a base pattern for a duration, then turns off
     * for a duration, repeating endlessly. The base pattern can be any
     * {@link LEDPattern} — solid, blinking, rainbow, chase, etc.
     *
     * <p>This differs from {@link LEDPattern#blink} in that the base pattern's
     * own animation continues to run during the "on" phase rather than being
     * reduced to a simple on/off toggle.
     *
     * @param base    The pattern to display during the "on" phase
     * @param onTime  Duration the base pattern is visible
     * @param offTime Duration the LEDs are off before repeating
     * @return A new {@link LEDPattern} implementing the on-off cycle
     */
    public static LEDPattern burstBlink(LEDPattern base, Time onTime, Time offTime) {
        long onMicros = (long) onTime.in(Microseconds);
        long totalCycleMicros = onMicros + (long) offTime.in(Microseconds);
        long[] startTime = { -1 };

        return (reader, writer) -> {
            long now = RobotController.getTime();
            if (startTime[0] < 0) {
                startTime[0] = now;
            }
            long t = (now - startTime[0]) % totalCycleMicros;
            if (t < onMicros) {
                base.applyTo(reader, writer);
            } else {
                LEDPattern.kOff.applyTo(reader, writer);
            }
        };
    }

    /**
     * Creates a pattern that shows a base pattern for a duration, then turns off
     * for a duration, repeating endlessly, with a one-time startup delay. The
     * LEDs stay off for the specified delay duration after the pattern first
     * starts, then begin the on/off cycle normally. The delay only applies once.
     *
     * @param base    The pattern to display during the "on" phase
     * @param onTime  Duration the base pattern is visible
     * @param offTime Duration the LEDs are off before repeating
     * @param delay   One-time delay before the burst cycle begins
     * @return A new {@link LEDPattern} implementing the delayed on-off cycle
     */
    public static LEDPattern burstBlink(LEDPattern base, Time onTime, Time offTime, Time delay) {
        long onMicros = (long) onTime.in(Microseconds);
        long totalCycleMicros = onMicros + (long) offTime.in(Microseconds);
        long delayMicros = (long) delay.in(Microseconds);
        long[] startTime = { -1 };

        return (reader, writer) -> {
            long now = RobotController.getTime();
            if (startTime[0] < 0) {
                startTime[0] = now;
            }
            long elapsed = now - startTime[0];
            if (elapsed < delayMicros) {
                LEDPattern.kOff.applyTo(reader, writer);
                return;
            }
            long t = (elapsed - delayMicros) % totalCycleMicros;
            if (t < onMicros) {
                base.applyTo(reader, writer);
            } else {
                LEDPattern.kOff.applyTo(reader, writer);
            }
        };
    }

    /**
     * Creates a two-tone alternating pattern. The strip is split in half: during
     * the first phase the left half shows the primary color and the right half
     * shows the secondary color; during the second phase they swap.
     *
     * @param primary   The color shown on the left half in phase A (right half in phase B)
     * @param secondary The color shown on the right half in phase A (left half in phase B)
     * @param period    How long each phase lasts before swapping
     * @return A new {@link LEDPattern} implementing the alternating two-tone behavior
     */
    public static LEDPattern twoTone(Color primary, Color secondary, Time period) {
        var patternA = LEDPattern.steps(dualColorMask(primary, secondary, 0.5));
        var patternB = LEDPattern.steps(dualColorMask(secondary, primary, 0.5));

        long periodMicros = (long) period.in(Microseconds);
        long totalCycleMicros = periodMicros * 2;

        return (reader, writer) -> {
            long t = RobotController.getTime() % totalCycleMicros;
            if (t < periodMicros) {
                patternA.applyTo(reader, writer);
            } else {
                patternB.applyTo(reader, writer);
            }
        };
    }

    /**
     * Creates a two-zone alternating pattern using arbitrary {@link LEDPattern}s.
     * The strip is split in half: during the first phase the left half renders
     * {@code left} and the right half renders {@code right}; during the second
     * phase they swap.
     *
     * <p>Unlike {@link #twoTone}, which only accepts solid colors, this method
     * allows full animated patterns (blink, rainbow, chase, etc.) on each half.
     *
     * @param left   The pattern shown on the left half in phase A (right half in phase B)
     * @param right  The pattern shown on the right half in phase A (left half in phase B)
     * @param period How long each phase lasts before swapping
     * @return A new {@link LEDPattern} implementing the alternating behavior
     */
    public static LEDPattern alternating(LEDPattern left, LEDPattern right, Time period) {
        long periodMicros = (long) period.in(Microseconds);
        long totalCycleMicros = periodMicros * 2;

        return (reader, writer) -> {
            int length = reader.getLength();
            int half = length / 2;
            long t = RobotController.getTime() % totalCycleMicros;
            boolean phaseA = t < periodMicros;

            LEDPattern leftPattern = phaseA ? left : right;
            LEDPattern rightPattern = phaseA ? right : left;

            // Apply left pattern to first half, right pattern to second half
            leftPattern.applyTo(reader, (index, r, g, b) -> {
                if (index < half) {
                    writer.setRGB(index, r, g, b);
                }
            });
            rightPattern.applyTo(reader, (index, r, g, b) -> {
                if (index >= half) {
                    writer.setRGB(index, r, g, b);
                }
            });
        };
    }

    /**
     * Splits the strip into two zones at a fixed pixel boundary and renders a
     * different pattern in each zone. The first {@code splitIndex} pixels display
     * {@code first}, and the remaining pixels display {@code second}.
     *
     * <p>Both patterns receive the full strip's {@link edu.wpi.first.wpilibj.LEDReader}
     * but writes are filtered to their respective zone, so animated patterns
     * (chase, rainbow, etc.) will still animate correctly within each zone.
     *
     * @param first      The pattern rendered on pixels {@code 0} to {@code splitIndex - 1}
     * @param second     The pattern rendered on pixels {@code splitIndex} to end
     * @param splitIndex The pixel index where the second zone begins (exclusive upper
     *                   bound of the first zone)
     * @return A new {@link LEDPattern} composing the two zones
     */
    public static LEDPattern split(LEDPattern first, LEDPattern second, int splitIndex) {
        return (reader, writer) -> {
            first.applyTo(reader, (index, r, g, b) -> {
                if (index < splitIndex) {
                    writer.setRGB(index, r, g, b);
                }
            });
            second.applyTo(reader, (index, r, g, b) -> {
                if (index >= splitIndex) {
                    writer.setRGB(index, r, g, b);
                }
            });
        };
    }

    // =========================== Color Mask Helpers ===========================

    /**
     * Creates a step mask with a single color at the given position and black elsewhere.
     *
     * @param color    The color for the mask
     * @param position The normalized position (0.0–1.0) where the color starts
     * @return A map suitable for {@link LEDPattern#steps}
     */
    public static Map<Number, Color> singleColorMask(Color color, Number position) {
        return dualColorMask(color, Color.kBlack, position);
    }

    /**
     * Creates a two-segment step mask: {@code color1} from 0 to {@code position},
     * then {@code color2} from {@code position} onward.
     *
     * @param color1   The color for the first segment
     * @param color2   The color for the second segment
     * @param position The normalized position (0.0–1.0) where the second segment begins
     * @return A map suitable for {@link LEDPattern#steps}
     */
    public static Map<Number, Color> dualColorMask(Color color1, Color color2, Number position) {
        return Map.of(0, color1, position, color2);
    }
}
