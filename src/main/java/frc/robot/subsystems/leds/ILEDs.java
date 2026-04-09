package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface for LED subsystems. Both {@link PwmLEDs} and
 * {@link frc.robot.subsystems.leds.prism.PrismLEDs PrismLEDs} implement this
 * so that {@code Container.LEDs} can be typed polymorphically.
 */
public interface ILEDs {

    /**
     * Sets the LED pattern for a specific section/strip.
     *
     * @param section The section index
     * @param pattern The pattern to display
     */
    void setSectionPattern(int section, LEDPattern pattern);

    /**
     * Sets the same LED pattern on all sections/strips.
     *
     * @param pattern The pattern to display on all sections
     */
    void setAllSectionPatterns(LEDPattern pattern);

    /**
     * Returns a command that sets the pattern on a specific section.
     *
     * @param section The section index
     * @param pattern The pattern to display
     * @return A command that sets the section pattern (runs while disabled)
     */
    Command setSectionPatternCommand(int section, LEDPattern pattern);

    /**
     * Returns a command that sets the pattern on all sections.
     *
     * @param pattern The pattern to display on all sections
     * @return A command that sets all section patterns (runs while disabled)
     */
    Command setAllSectionPatternsCommand(LEDPattern pattern);
}
