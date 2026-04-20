package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Container;
import frc.robot.subsystems.leds.LEDPatterns;

/**
 * Resolves LED patterns for strip 1 based on swerve drive state.
 * <ul>
 *   <li><b>Auto-align active</b>: yellow fast blink (locking heading)</li>
 *   <li><b>Autonomous driving</b>: green alternating fast (path following)</li>
 *   <li><b>Robot-relative driving</b>: yellow alternating slow (caution)</li>
 *   <li><b>Normal teleop driving</b>: blue slow blink (field-relative)</li>
 *   <li><b>Disabled</b>: null (no opinion)</li>
 * </ul>
 */
public class SwerveLEDHelper {

    private static final int STRIP_INDEX = 1;
    private static LEDPattern _lastPattern = null;

    /**
     * Resolves the desired LED pattern for the swerve drive's current state.
     * Returns null when disabled (no swerve LED opinion).
     */
    public static LEDPattern resolveDesiredLedPattern(SwerveSubsystemInputsAutoLogged inputs) {
        if (DriverStation.isDisabled()) {
            return null;
        }

        if (inputs.UseAutoAlign) {
            return LEDPatterns.YellowFastBlink;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return LEDPatterns.GreenAlternatingFast;
        }

        if (inputs.DrivingRobotRelative) {
            return LEDPatterns.YellowAlternatingSlow;
        }

        // Normal teleop field-relative driving
        return LEDPatterns.BlueSlowBlink;
    }

    /**
     * Updates LED strip 1 only when the desired pattern changes.
     */
    public static void updateLEDs(SwerveSubsystemInputsAutoLogged inputs) {
        if (Container.LEDs == null)
            return;

        LEDPattern desiredPattern = resolveDesiredLedPattern(inputs);
        if (desiredPattern != null && desiredPattern != _lastPattern) {
            Container.LEDs.setSectionPattern(STRIP_INDEX, desiredPattern);
            _lastPattern = desiredPattern;
        }
    }
}
