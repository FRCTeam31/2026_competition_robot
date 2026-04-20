package frc.robot.subsystems.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Container;
import frc.robot.Robot;

/**
 * Resolves LED patterns for strip 3 based on overall match/robot state.
 * <ul>
 *   <li><b>Disabled</b>: alliance color breathe</li>
 *   <li><b>Autonomous enabled</b>: green-red alternating (auto indicator)</li>
 *   <li><b>Teleop — endgame (≤30s remaining)</b>: red quick flash (endgame warning)</li>
 *   <li><b>Teleop — normal</b>: alliance color slow blink</li>
 *   <li><b>Estopped</b>: red fast blink</li>
 * </ul>
 */
public class MatchLEDHelper {

    private static final int STRIP_INDEX = 3;
    private static final double ENDGAME_THRESHOLD_SECONDS = 30.0;

    private static LEDPattern _lastPattern = null;

    /**
     * Resolves the desired LED pattern based on current match state.
     */
    public static LEDPattern resolveDesiredLedPattern() {
        if (DriverStation.isEStopped()) {
            return LEDPatterns.RedFastBlink;
        }

        Color allianceColor = Robot.getAllianceColor();

        if (DriverStation.isDisabled()) {
            return LEDPattern.solid(allianceColor).breathe(Units.Seconds.of(3));
        }

        if (DriverStation.isAutonomousEnabled()) {
            return LEDPatterns.GreenRedAlternating;
        }

        // Teleop
        if (DriverStation.isTeleopEnabled()) {
            double matchTime = DriverStation.getMatchTime();
            // matchTime returns -1 when not in a real match; only trigger endgame in a real match
            if (matchTime >= 0 && matchTime <= ENDGAME_THRESHOLD_SECONDS) {
                return LEDPatterns.RedQuickFlash;
            }

            return LEDPatterns.slowBlink(allianceColor);
        }

        // Test mode or unknown
        return LEDPatterns.BlueYellowAlternating;
    }

    /**
     * Updates LED strip 3 only when the desired pattern changes.
     * Should be called from {@code robotPeriodic()}.
     */
    public static void updateLEDs() {
        if (Container.LEDs == null)
            return;

        LEDPattern desiredPattern = resolveDesiredLedPattern();
        if (desiredPattern != null && desiredPattern != _lastPattern) {
            Container.LEDs.setSectionPattern(STRIP_INDEX, desiredPattern);
            _lastPattern = desiredPattern;
        }
    }
}
