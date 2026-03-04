package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

/**
 * Unit tests for Swerve subsystem - validates subsystem initialization and command creation.
 */
class SwerveTest {
    private Swerve swerve;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        swerve = new Swerve();
    }

    @Test
    void testConstructor_InitializesBothModes() {
        assertDoesNotThrow(() -> new Swerve(), "Should initialize without errors");
    }

    @Test
    void testConstructor_SetsSubsystemName() {
        assertEquals("Swerve", swerve.getName());
    }

    @Test
    void testCommandFactories_ReturnValidCommands() {
        assertNotNull(swerve.resetGyroCommand());
        assertNotNull(swerve.stopAllMotorsCommand());
        assertNotNull(swerve.disableAutoAlignCommand());
        assertNotNull(swerve.setAutoAlignSetpointCommand(90));
        assertNotNull(swerve.enablePathPlannerAutoAlignRotationFeedbackCommand());
        assertNotNull(swerve.disablePathPlannerAutoAlignRotationFeedbackCommand());
        assertNotNull(swerve.cancelPathfindingCommand());
        assertNotNull(swerve.faceAwayFromHubCommand());
    }

    @Test
    void testCommandFactories_ReturnNewInstances() {
        var cmd1 = swerve.resetGyroCommand();
        var cmd2 = swerve.resetGyroCommand();
        assertNotSame(cmd1, cmd2, "Each command creation should return a new instance");
    }

    @Test
    void testSetAutoAlignSetpoint_AcceptsEdgeCases() {
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(0));
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(360));
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(-360));
        assertDoesNotThrow(() -> swerve.setAutoAlignSetpointCommand(720));
    }
}
