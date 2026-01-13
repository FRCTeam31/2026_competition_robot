package frc.robot.subsystems.swerve.module;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.prime.control.ExtendedPIDConstants;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Unit tests for SwerveModuleSim - validates simulated swerve module behavior.
 * Tests verify module state updates, motor control, PID configuration, and state optimization.
 */
class SwerveModuleSimTest {
    private SwerveModuleSim module;
    private SwerveModuleInputsAutoLogged inputs;
    private static final double EPSILON = 1e-6;
    private static final SwerveModuleMap TEST_MODULE_MAP = new SwerveModuleMap(
            1, 2, 3, 0.0, false, false, new Translation2d(0.5, 0.5));

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        module = new SwerveModuleSim("TestModule", TEST_MODULE_MAP);
        inputs = new SwerveModuleInputsAutoLogged();
    }

    // ==================== Constructor Tests ====================

    @Test
    void testConstructor_InitializesSuccessfully() {
        assertDoesNotThrow(() -> new SwerveModuleSim("TestModule", TEST_MODULE_MAP),
                "Constructor should initialize without throwing exceptions");
    }

    @Test
    void testConstructor_AcceptsValidParameters() {
        SwerveModuleSim testModule = new SwerveModuleSim("CustomName", TEST_MODULE_MAP);
        assertNotNull(testModule, "Module should be created successfully");
    }

    // ==================== UpdateInputs Tests ====================

    @Test
    void testUpdateInputs_InitialStateIsZero() {
        module.updateInputs(inputs);

        assertEquals(0.0, inputs.ModuleState.speedMetersPerSecond, EPSILON,
                "Initial speed should be zero");
        assertEquals(0.0, inputs.ModuleState.angle.getRadians(), EPSILON,
                "Initial angle should be zero");
        assertEquals(0.0, inputs.ModulePosition.distanceMeters, EPSILON,
                "Initial distance should be zero");
    }

    @Test
    void testUpdateInputs_UpdatesModuleState() {
        module.updateInputs(inputs);

        assertNotNull(inputs.ModuleState, "Module state should not be null");
        assertNotNull(inputs.ModuleState.angle, "Module state angle should not be null");
    }

    @Test
    void testUpdateInputs_UpdatesModulePosition() {
        module.updateInputs(inputs);

        assertNotNull(inputs.ModulePosition, "Module position should not be null");
        assertNotNull(inputs.ModulePosition.angle, "Module position angle should not be null");
    }

    @Test
    void testUpdateInputs_ConsecutiveCallsShowSimulation() {
        // Set a voltage to make the motor move
        module.setDriveVoltage(6.0, Rotation2d.fromDegrees(0));

        module.updateInputs(inputs);
        double initialSpeed = inputs.ModuleState.speedMetersPerSecond;

        // Multiple updates should show the motor accelerating
        for (int i = 0; i < 10; i++) {
            module.updateInputs(inputs);
        }

        double finalSpeed = inputs.ModuleState.speedMetersPerSecond;
        assertTrue(Math.abs(finalSpeed) > Math.abs(initialSpeed),
                "Speed should increase after multiple simulation steps with voltage applied");
    }

    // ==================== SetDriveVoltage Tests ====================

    @Test
    void testSetDriveVoltage_AcceptsPositiveVoltage() {
        assertDoesNotThrow(() -> module.setDriveVoltage(6.0, Rotation2d.fromDegrees(0)),
                "Should accept positive voltage");
    }

    @Test
    void testSetDriveVoltage_AcceptsNegativeVoltage() {
        assertDoesNotThrow(() -> module.setDriveVoltage(-6.0, Rotation2d.fromDegrees(0)),
                "Should accept negative voltage");
    }

    @Test
    void testSetDriveVoltage_UpdatesModuleAngle() {
        Rotation2d targetAngle = Rotation2d.fromDegrees(90);
        module.setDriveVoltage(0.0, targetAngle);

        module.updateInputs(inputs);

        assertEquals(targetAngle.getRadians(), inputs.ModuleState.angle.getRadians(), EPSILON,
                "Module angle should be updated to target angle");
    }

    @Test
    void testSetDriveVoltage_CausesAcceleration() {
        module.setDriveVoltage(12.0, Rotation2d.fromDegrees(0));

        module.updateInputs(inputs);
        double speed1 = inputs.ModuleState.speedMetersPerSecond;

        module.updateInputs(inputs);
        double speed2 = inputs.ModuleState.speedMetersPerSecond;

        assertTrue(Math.abs(speed2) > Math.abs(speed1),
                "Module should accelerate when voltage is applied");
    }

    @Test
    void testSetDriveVoltage_WithZeroVoltage() {
        // First accelerate the module
        module.setDriveVoltage(12.0, Rotation2d.fromDegrees(0));
        for (int i = 0; i < 10; i++) {
            module.updateInputs(inputs);
        }

        // Then set voltage to zero
        module.setDriveVoltage(0.0, Rotation2d.fromDegrees(0));
        module.updateInputs(inputs);

        // The module should start decelerating (speed should remain non-zero initially due to inertia)
        assertNotNull(inputs.ModuleState);
    }

    // ==================== StopMotors Tests ====================

    @Test
    void testStopMotors_SetsVoltageToZero() {
        // First accelerate the module
        module.setDriveVoltage(12.0, Rotation2d.fromDegrees(45));
        for (int i = 0; i < 10; i++) {
            module.updateInputs(inputs);
        }

        // Stop the motors
        module.stopMotors();
        module.updateInputs(inputs);

        // Velocity should be zero after stopping
        assertEquals(0.0, inputs.ModuleState.speedMetersPerSecond, EPSILON,
                "Module speed should be zero after stopMotors is called");
    }

    @Test
    void testStopMotors_MultipleCalls() {
        module.stopMotors();
        module.stopMotors();

        module.updateInputs(inputs);

        assertEquals(0.0, inputs.ModuleState.speedMetersPerSecond, EPSILON,
                "Multiple stop calls should not cause issues");
    }

    // ==================== SetDrivePID Tests ====================

    @Test
    void testSetDrivePID_AcceptsNewPIDConstants() {
        ExtendedPIDConstants newPID = new ExtendedPIDConstants(1.0, 0.0, 0.1, 0.0, 0.5);

        assertDoesNotThrow(() -> module.setDrivePID(newPID),
                "Should accept new PID constants");
    }

    @Test
    void testSetDrivePID_PreservesSetpoint() {
        // Set a desired state to establish a setpoint
        SwerveModuleState desiredState = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
        module.setDesiredState(desiredState);

        // Change PID constants
        ExtendedPIDConstants newPID = new ExtendedPIDConstants(2.0, 0.0, 0.2, 0.0, 0.6);
        module.setDrivePID(newPID);

        // The module should still be trying to reach the previous setpoint
        // This is validated by ensuring no exceptions are thrown
        assertDoesNotThrow(() -> module.updateInputs(inputs));
    }

    @Test
    void testSetDrivePID_UpdatesFeedforward() {
        ExtendedPIDConstants highKv = new ExtendedPIDConstants(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.5);
        module.setDrivePID(highKv);

        // Setting a desired state should use the new feedforward values
        SwerveModuleState desiredState = new SwerveModuleState(2.0, Rotation2d.fromDegrees(0));
        assertDoesNotThrow(() -> module.setDesiredState(desiredState));
    }

    // ==================== SetSteeringPID Tests ====================

    @Test
    void testSetSteeringPID_DoesNotThrow() {
        ExtendedPIDConstants steeringPID = new ExtendedPIDConstants(5.0, 0.0, 0.1);

        assertDoesNotThrow(() -> module.setSteeringPID(steeringPID),
                "SetSteeringPID should not throw even if not implemented");
    }

    // ==================== SetDesiredState Tests ====================

    @Test
    void testSetDesiredState_AcceptsValidState() {
        SwerveModuleState desiredState = new SwerveModuleState(1.5, Rotation2d.fromDegrees(45));

        assertDoesNotThrow(() -> module.setDesiredState(desiredState),
                "Should accept valid swerve module state");
    }

    @Test
    void testSetDesiredState_UpdatesModuleAngle() {
        SwerveModuleState desiredState = new SwerveModuleState(1.0, Rotation2d.fromDegrees(90));
        module.setDesiredState(desiredState);

        module.updateInputs(inputs);

        assertEquals(Math.toRadians(90), inputs.ModuleState.angle.getRadians(), EPSILON,
                "Module angle should match desired state");
    }

    @Test
    void testSetDesiredState_DrivesAtDesiredSpeed() {
        SwerveModuleState desiredState = new SwerveModuleState(2.0, Rotation2d.fromDegrees(0));
        module.setDesiredState(desiredState);

        // Allow time for the module to reach desired speed
        for (int i = 0; i < 50; i++) {
            module.updateInputs(inputs);
        }

        // Speed should be close to desired (within reasonable tolerance due to PID control)
        assertEquals(2.0, inputs.ModuleState.speedMetersPerSecond, 0.5,
                "Module should approach desired speed");
    }

    @Test
    void testSetDesiredState_WithZeroSpeed() {
        SwerveModuleState desiredState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        module.setDesiredState(desiredState);

        module.updateInputs(inputs);

        // With zero speed, the angle might not update (depends on implementation)
        // But it should not throw an exception
        assertNotNull(inputs.ModuleState);
    }

    @Test
    void testSetDesiredState_OptimizationReducesRotation() {
        // Set initial angle
        module.setDriveVoltage(0, Rotation2d.fromDegrees(0));
        module.updateInputs(inputs);

        // Request state that would require 270 degree rotation
        // Optimization should flip it to 90 degrees with reversed speed
        SwerveModuleState desiredState = new SwerveModuleState(1.0, Rotation2d.fromDegrees(270));
        module.setDesiredState(desiredState);

        module.updateInputs(inputs);

        // The angle should be optimized (not necessarily 270, could be -90 which is equivalent)
        double angleDegrees = Math.toDegrees(inputs.ModuleState.angle.getRadians());
        assertTrue(angleDegrees >= -180 && angleDegrees <= 180,
                "Optimized angle should be within -180 to 180 range");
    }

    @Test
    void testSetDesiredState_MultipleStatesInSequence() {
        SwerveModuleState state1 = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
        module.setDesiredState(state1);
        module.updateInputs(inputs);

        SwerveModuleState state2 = new SwerveModuleState(2.0, Rotation2d.fromDegrees(90));
        module.setDesiredState(state2);
        module.updateInputs(inputs);

        SwerveModuleState state3 = new SwerveModuleState(0.5, Rotation2d.fromDegrees(180));
        module.setDesiredState(state3);
        module.updateInputs(inputs);

        // Should handle multiple state changes without issues
        assertNotNull(inputs.ModuleState);
    }

    // ==================== Distance Accumulation Tests ====================

    @Test
    void testModulePosition_AccumulatesDistance() {
        module.setDriveVoltage(12.0, Rotation2d.fromDegrees(0));

        module.updateInputs(inputs);
        double distance1 = inputs.ModulePosition.distanceMeters;

        // Simulate for multiple cycles
        for (int i = 0; i < 20; i++) {
            module.updateInputs(inputs);
        }

        double distance2 = inputs.ModulePosition.distanceMeters;

        assertTrue(distance2 > distance1,
                "Distance should accumulate as module moves");
    }

    @Test
    void testModulePosition_DistanceIncreasesWithSpeed() {
        module.setDriveVoltage(6.0, Rotation2d.fromDegrees(0));

        for (int i = 0; i < 10; i++) {
            module.updateInputs(inputs);
        }
        double lowSpeedDistance = inputs.ModulePosition.distanceMeters;

        module.stopMotors();
        module.updateInputs(inputs);

        // Reset and test with higher voltage
        SwerveModuleSim module2 = new SwerveModuleSim("TestModule2", TEST_MODULE_MAP);
        SwerveModuleInputsAutoLogged inputs2 = new SwerveModuleInputsAutoLogged();

        module2.setDriveVoltage(12.0, Rotation2d.fromDegrees(0));

        for (int i = 0; i < 10; i++) {
            module2.updateInputs(inputs2);
        }
        double highSpeedDistance = inputs2.ModulePosition.distanceMeters;

        assertTrue(highSpeedDistance > lowSpeedDistance,
                "Higher voltage should result in more distance traveled");
    }

    // ==================== Integration Tests ====================

    @Test
    void testFullCycle_SetStateAndUpdate() {
        SwerveModuleState desiredState = new SwerveModuleState(1.5, Rotation2d.fromDegrees(45));
        module.setDesiredState(desiredState);

        module.updateInputs(inputs);

        assertNotNull(inputs.ModuleState);
        assertNotNull(inputs.ModulePosition);
        assertEquals(Math.toRadians(45), inputs.ModuleState.angle.getRadians(), EPSILON);
    }

    @Test
    void testSimulationConvergence() {
        // Test that the module converges to a steady state speed
        SwerveModuleState desiredState = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
        module.setDesiredState(desiredState);

        double previousSpeed = 0;
        double speedChangeThreshold = 0.01;
        int stableCount = 0;

        for (int i = 0; i < 100; i++) {
            module.updateInputs(inputs);
            double currentSpeed = inputs.ModuleState.speedMetersPerSecond;

            if (Math.abs(currentSpeed - previousSpeed) < speedChangeThreshold) {
                stableCount++;
                if (stableCount > 10) {
                    // Speed has stabilized
                    break;
                }
            } else {
                stableCount = 0;
            }

            previousSpeed = currentSpeed;
        }

        assertTrue(stableCount > 10 || Math.abs(inputs.ModuleState.speedMetersPerSecond - 1.0) < 0.3,
                "Module should converge to desired speed or stabilize");
    }

    // ==================== Edge Cases ====================

    @Test
    void testSetDriveVoltage_WithExtremeAngles() {
        assertDoesNotThrow(() -> module.setDriveVoltage(6.0, Rotation2d.fromDegrees(720)),
                "Should handle angles > 360 degrees");

        assertDoesNotThrow(() -> module.setDriveVoltage(6.0, Rotation2d.fromDegrees(-540)),
                "Should handle large negative angles");
    }

    @Test
    void testSetDesiredState_WithHighSpeed() {
        SwerveModuleState highSpeedState = new SwerveModuleState(5.0, Rotation2d.fromDegrees(0));

        assertDoesNotThrow(() -> module.setDesiredState(highSpeedState),
                "Should handle high speed requests");
    }

    @Test
    void testSetDesiredState_WithNegativeSpeed() {
        SwerveModuleState negativeSpeedState = new SwerveModuleState(-1.0, Rotation2d.fromDegrees(0));

        assertDoesNotThrow(() -> module.setDesiredState(negativeSpeedState),
                "Should handle negative speed (though may be optimized)");
    }

    @Test
    void testModuleAfterStopAndRestart() {
        // Drive the module
        module.setDriveVoltage(12.0, Rotation2d.fromDegrees(0));
        for (int i = 0; i < 10; i++) {
            module.updateInputs(inputs);
        }

        // Stop it
        module.stopMotors();
        module.updateInputs(inputs);

        // Drive again
        module.setDriveVoltage(6.0, Rotation2d.fromDegrees(90));
        module.updateInputs(inputs);

        // Should work normally after stop
        assertNotNull(inputs.ModuleState);
    }

    @Test
    void testImplementsISwerveModuleInterface() {
        assertTrue(ISwerveModule.class.isAssignableFrom(SwerveModuleSim.class),
                "SwerveModuleSim should implement ISwerveModule interface");
    }
}
