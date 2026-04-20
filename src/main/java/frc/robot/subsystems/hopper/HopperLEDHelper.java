package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Container;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;
import frc.robot.subsystems.hopper.Hopper.TransferFeedState;
import frc.robot.subsystems.leds.LEDPatterns;

/**
 * Resolves LED patterns for strip 2 based on hopper/intake state.
 * <ul>
 *   <li><b>Transfer feeding inwards</b>: green fast blink (feeding to shooter)</li>
 *   <li><b>Transfer feeding outwards</b>: red fast blink (ejecting)</li>
 *   <li><b>Intake feeding inwards</b>: yellow alternating slow (intaking)</li>
 *   <li><b>Intake feeding outwards</b>: red alternating slow (reversing intake)</li>
 *   <li><b>All stopped</b>: green slow blink (idle / ready)</li>
 * </ul>
 */
public class HopperLEDHelper {

    private static final int STRIP_INDEX = 2;
    private static LEDPattern _lastPattern = null;

    /**
     * Resolves the desired LED pattern for the hopper's current state.
     */
    public static LEDPattern resolveDesiredLedPattern(HopperInputsAutoLogged inputs) {
        // Transfer feed takes priority — it means we're actively shooting or ejecting
        if (inputs.TransferFeedState == TransferFeedState.INWARDS) {
            return LEDPatterns.GreenFastBlink;
        } else if (inputs.TransferFeedState == TransferFeedState.OUTWARDS) {
            return LEDPatterns.RedFastBlink;
        }

        // Intake feed
        if (inputs.IntakeFeedState == IntakeFeedState.INWARDS) {
            return LEDPatterns.YellowAlternatingSlow;
        } else if (inputs.IntakeFeedState == IntakeFeedState.OUTWARDS) {
            return LEDPatterns.RedAlternatingSlow;
        }

        // Everything stopped — idle
        return LEDPatterns.GreenSlowBlink;
    }

    /**
     * Updates LED strip 2 only when the desired pattern changes.
     */
    public static void updateLEDs(HopperInputsAutoLogged inputs) {
        if (Container.LEDs == null)
            return;

        LEDPattern desiredPattern = resolveDesiredLedPattern(inputs);
        if (desiredPattern != null && desiredPattern != _lastPattern) {
            Container.LEDs.setSectionPattern(STRIP_INDEX, desiredPattern);
            _lastPattern = desiredPattern;
        }
    }
}
