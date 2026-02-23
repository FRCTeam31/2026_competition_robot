package org.prime.sysid;

import java.util.function.Consumer;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Helper class for creating WPILib SysId characterization routines.
 * Provides a clean API with enums for test type and direction,
 * encapsulating the SysIdRoutine setup pattern.
 */
public class SysIdRoutineHelper {

    /** The type of SysId test to run */
    public enum TestType {
        /** Slowly ramps voltage to measure static and velocity gains (kS, kV) */
        QUASISTATIC,
        /** Applies a step voltage to measure acceleration gain (kA) */
        DYNAMIC
    }

    /** The direction of the SysId test */
    public enum TestDirection {
        /** Run the mechanism forward (positive voltage) */
        FORWARD,
        /** Run the mechanism in reverse (negative voltage) */
        REVERSE
    }

    private final SysIdRoutine _routine;

    /**
     * Creates a SysIdRoutineHelper for a mechanism.
     *
     * @param subsystem     The subsystem that owns this mechanism (for command requirements)
     * @param mechanismName A human-readable name for the mechanism (e.g., "Flywheel", "TurretYaw", "SwerveDrive")
     * @param driveConsumer A consumer that applies voltage to the mechanism's motor(s)
     * @param logConsumer   A consumer that logs the mechanism's current state (voltage, position, velocity)
     */
    public SysIdRoutineHelper(
            SubsystemBase subsystem,
            String mechanismName,
            Consumer<Voltage> driveConsumer,
            Consumer<SysIdRoutineLog> logConsumer) {

        _routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        driveConsumer,
                        logConsumer,
                        subsystem,
                        mechanismName));
    }

    /**
     * Creates a SysIdRoutineHelper with custom SysId configuration.
     *
     * @param subsystem     The subsystem that owns this mechanism
     * @param mechanismName A human-readable name for the mechanism
     * @param config        Custom SysIdRoutine configuration (ramp rate, step voltage, timeout)
     * @param driveConsumer A consumer that applies voltage to the mechanism's motor(s)
     * @param logConsumer   A consumer that logs the mechanism's current state
     */
    public SysIdRoutineHelper(
            SubsystemBase subsystem,
            String mechanismName,
            SysIdRoutine.Config config,
            Consumer<Voltage> driveConsumer,
            Consumer<SysIdRoutineLog> logConsumer) {

        _routine = new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        driveConsumer,
                        logConsumer,
                        subsystem,
                        mechanismName));
    }

    /**
     * Returns a command to run the specified SysId routine.
     *
     * @param testType  Whether to run a quasistatic or dynamic test
     * @param direction Whether to run the mechanism forward or in reverse
     * @return A command that runs the requested SysId test
     */
    public Command getCommand(TestType testType, TestDirection direction) {
        var sysIdDirection = direction == TestDirection.FORWARD
                ? SysIdRoutine.Direction.kForward
                : SysIdRoutine.Direction.kReverse;

        if (testType == TestType.QUASISTATIC) {
            return _routine.quasistatic(sysIdDirection);
        } else {
            return _routine.dynamic(sysIdDirection);
        }
    }
}
