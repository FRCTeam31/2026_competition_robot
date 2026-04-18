package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Container;
import frc.robot.subsystems.leds.LEDPatterns;
import frc.robot.subsystems.turret.Turret.FiringState;
import frc.robot.subsystems.turret.Turret.ShotState;
import frc.robot.subsystems.turret.Turret.OperatingMode;

public class TurretLEDHelper {

    // LED state tracking - only update LEDs on state transitions
    private static LEDPattern _lastLedPattern = null;

    /**
     * Resolves the desired LED pattern for the turret's current state.
     * Returns null for STOPPED mode (no turret LED opinion).
     */
    public static LEDPattern resolveDesiredLedPattern(TurretInputsAutoLogged inputs) {
        boolean firing = inputs.FiringState == FiringState.FIRING;
        boolean allOnTarget = inputs.FlywheelAtTargetSpeed && inputs.YawOnTarget && inputs.HoodOnTarget;
        boolean shotFailed = inputs.ShotCalculationState == ShotState.SHOT_NOT_CALCULATED;

        if (inputs.OperatingMode == OperatingMode.AUTO) {
            if (firing && allOnTarget) {
                // If we're firing and all systems are go, we're shooting!
                return LEDPatterns.GreenAlternatingFast;
            } else if (firing && shotFailed) {
                // If we're firing but shot isn't calculable, we likely have a constraint failure (e.g. target too close)
                return LEDPatterns.RedQuickFlash;
            } else if (firing) {
                // If we're firing but not all on target and shot is calculable, we're likely still seeking or aligning
                return LEDPatterns.BlueFastBlink;
            } else {
                // If we're not firing, we're idle
                return LEDPatterns.BlueAlternatingSlow;
            }
        } else if (inputs.OperatingMode == OperatingMode.MANUAL) {
            if (firing) {
                // If we're firing in manual mode, we're shooting!
                return LEDPatterns.YellowAlternatingFast;
            } else {
                // If we're not firing in manual mode, we're idle
                return LEDPatterns.YellowAlternatingSlow;
            }
        }

        return null;
    }

    /**
     * Updates LED patterns only when the turret's desired pattern changes,
     * preventing continuous overrides of patterns set by other subsystems.
     * <ul>
     *   <li><b>AUTO + FIRING + all on-target</b>: green two-tone fast (shooting!)</li>
     *   <li><b>AUTO + FIRING + shot not calculable</b>: red quick flash (constraint failure)</li>
     *   <li><b>AUTO + FIRING + seeking</b>: yellow fast blink (locking on)</li>
     *   <li><b>AUTO + IDLE</b>: blue slow blink (ready, auto mode)</li>
     *   <li><b>MANUAL + FIRING</b>: yellow two-tone fast (manual shooting)</li>
     *   <li><b>MANUAL + IDLE</b>: blue fast blink (ready, manual mode)</li>
     * </ul>
     */
    public static void updateLEDs(TurretInputsAutoLogged inputs) {
        if (Container.LEDs == null)
            return;

        LEDPattern desiredPattern = resolveDesiredLedPattern(inputs);
        if (desiredPattern != null && desiredPattern != _lastLedPattern) {
            Container.LEDs.setAllSectionPatterns(desiredPattern);
            _lastLedPattern = desiredPattern;
        }
    }
}
