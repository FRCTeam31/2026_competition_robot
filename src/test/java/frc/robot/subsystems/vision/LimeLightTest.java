package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.mockito.ArgumentMatchers.doubleThat;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.MockedStatic;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.helpers.LimelightHelpers;

/**
 * Unit tests for the LimeLight class.
 * Tests LED control, pipeline management, streaming modes,
 * camera pose configuration, and targeting data retrieval.
 */
class LimeLightTest {
    private LimeLight limelight;
    private MockedStatic<LimelightHelpers> mockHelpers;
    private static final String TEST_LIMELIGHT_NAME = "test-limelight";

    @BeforeEach
    void setUp() {
        // Initialize HAL for WPILib
        assert HAL.initialize(500, 0);

        // Mock LimelightHelpers static methods
        mockHelpers = mockStatic(LimelightHelpers.class);

        limelight = new LimeLight(TEST_LIMELIGHT_NAME);
    }

    @AfterEach
    void tearDown() {
        if (limelight != null) {
            limelight.close();
        }
        if (mockHelpers != null) {
            mockHelpers.close();
        }
    }

    //#region Constructor Tests

    @Test
    void testConstructorWithValidName() {
        LimeLight ll = new LimeLight("valid-name");
        assertNotNull(ll, "LimeLight should be constructed successfully");
        ll.close();
    }

    @Test
    void testConstructorWithEmptyName() {
        LimeLight ll = new LimeLight("");
        assertNotNull(ll, "LimeLight should accept empty name");
        ll.close();
    }

    //#endregion

    //#region LED Mode Tests

    @Test
    void testSetLedModePipelineControl() {
        limelight.setLedMode(0);

        mockHelpers.verify(() -> LimelightHelpers.setLEDMode_PipelineControl(TEST_LIMELIGHT_NAME), times(1));
    }

    @Test
    void testSetLedModeForceOff() {
        limelight.setLedMode(1);

        mockHelpers.verify(() -> LimelightHelpers.setLEDMode_ForceOff(TEST_LIMELIGHT_NAME), times(1));
    }

    @Test
    void testSetLedModeForceBlink() {
        limelight.setLedMode(2);

        mockHelpers.verify(() -> LimelightHelpers.setLEDMode_ForceBlink(TEST_LIMELIGHT_NAME), times(1));
    }

    @Test
    void testSetLedModeForceOn() {
        limelight.setLedMode(3);

        mockHelpers.verify(() -> LimelightHelpers.setLEDMode_ForceOn(TEST_LIMELIGHT_NAME), times(1));
    }

    @ParameterizedTest
    @ValueSource(ints = { -1, -5, 4, 10, 100 })
    void testSetLedModeInvalidValues(int invalidMode) {
        assertThrows(IllegalArgumentException.class,
                () -> limelight.setLedMode(invalidMode),
                "Setting invalid LED mode should throw IllegalArgumentException");
    }

    @Test
    void testSetLedModeInvalidThrowsCorrectMessage() {
        Exception exception = assertThrows(IllegalArgumentException.class,
                () -> limelight.setLedMode(5));

        assertEquals("LED mode must be between 0 and 3", exception.getMessage(),
                "Exception message should indicate valid range");
    }

    //#endregion

    //#region Blink LED Tests

    @Test
    void testBlinkLedDoesNotThrowException() {
        assertDoesNotThrow(() -> limelight.blinkLed(3),
                "blinkLed should not throw exception");
    }

    @Test
    void testBlinkLedZeroCount() {
        assertDoesNotThrow(() -> limelight.blinkLed(0),
                "blinkLed with 0 count should not throw exception");
    }

    @Test
    void testBlinkLedLargeCount() {
        assertDoesNotThrow(() -> limelight.blinkLed(100),
                "blinkLed with large count should not throw exception");
    }

    //#endregion

    //#region Pipeline Tests

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 })
    void testSetPipelineValidPipelines(int pipeline) {
        limelight.setPipeline(pipeline);

        mockHelpers.verify(() -> LimelightHelpers.setPipelineIndex(TEST_LIMELIGHT_NAME, pipeline), times(1));
    }

    @ParameterizedTest
    @ValueSource(ints = { -1, -10, 10, 11, 100 })
    void testSetPipelineInvalidValues(int invalidPipeline) {
        assertThrows(IllegalArgumentException.class,
                () -> limelight.setPipeline(invalidPipeline),
                "Setting invalid pipeline should throw IllegalArgumentException");
    }

    @Test
    void testSetPipelineInvalidThrowsCorrectMessage() {
        Exception exception = assertThrows(IllegalArgumentException.class,
                () -> limelight.setPipeline(10));

        assertEquals("Pipeline must be between 0 and 9", exception.getMessage(),
                "Exception message should indicate valid range");
    }

    //#endregion

    //#region Streaming Mode Tests

    @Test
    void testSetPiPStreamingModeStandard() {
        limelight.setPiPStreamingMode(0);

        mockHelpers.verify(() -> LimelightHelpers.setStreamMode_Standard(TEST_LIMELIGHT_NAME), times(1));
    }

    @Test
    void testSetPiPStreamingModePiPMain() {
        limelight.setPiPStreamingMode(1);

        mockHelpers.verify(() -> LimelightHelpers.setStreamMode_PiPMain(TEST_LIMELIGHT_NAME), times(1));
    }

    @Test
    void testSetPiPStreamingModePiPSecondary() {
        limelight.setPiPStreamingMode(2);

        mockHelpers.verify(() -> LimelightHelpers.setStreamMode_PiPSecondary(TEST_LIMELIGHT_NAME), times(1));
    }

    @ParameterizedTest
    @ValueSource(ints = { -1, -5, 3, 10, 100 })
    void testSetPiPStreamingModeInvalidValues(int invalidMode) {
        assertThrows(IllegalArgumentException.class,
                () -> limelight.setPiPStreamingMode(invalidMode),
                "Setting invalid streaming mode should throw IllegalArgumentException");
    }

    @Test
    void testSetPiPStreamingModeInvalidThrowsCorrectMessage() {
        Exception exception = assertThrows(IllegalArgumentException.class,
                () -> limelight.setPiPStreamingMode(5));

        assertEquals("Streaming mode must be between 0 and 2", exception.getMessage(),
                "Exception message should indicate valid range");
    }

    //#endregion

    //#region Camera Pose Tests

    @Test
    void testSetCameraPoseBasic() {
        Pose3d testPose = new Pose3d(
                new Translation3d(1.0, 2.0, 3.0),
                new Rotation3d(0.1, 0.2, 0.3));

        limelight.setCameraPose(testPose);

        mockHelpers.verify(() -> LimelightHelpers.setCameraPose_RobotSpace(
                eq(TEST_LIMELIGHT_NAME),
                eq(1.0),
                eq(2.0),
                eq(3.0),
                doubleThat(val -> Math.abs(val - Units.radiansToDegrees(0.1)) < 0.0001),
                doubleThat(val -> Math.abs(val - Units.radiansToDegrees(0.2)) < 0.0001),
                doubleThat(val -> Math.abs(val - Units.radiansToDegrees(0.3)) < 0.0001)), times(1));
    }

    @Test
    void testSetCameraPoseZero() {
        Pose3d zeroPose = new Pose3d();

        limelight.setCameraPose(zeroPose);

        mockHelpers.verify(() -> LimelightHelpers.setCameraPose_RobotSpace(
                eq(TEST_LIMELIGHT_NAME),
                eq(0.0),
                eq(0.0),
                eq(0.0),
                eq(0.0),
                eq(0.0),
                eq(0.0)), times(1));
    }

    @Test
    void testSetCameraPoseNegativeValues() {
        Pose3d negativePose = new Pose3d(
                new Translation3d(-1.5, -2.5, -0.5),
                new Rotation3d(-Math.PI / 4, -Math.PI / 6, -Math.PI / 3));

        limelight.setCameraPose(negativePose);

        mockHelpers.verify(() -> LimelightHelpers.setCameraPose_RobotSpace(
                eq(TEST_LIMELIGHT_NAME),
                eq(-1.5),
                eq(-2.5),
                eq(-0.5),
                doubleThat(val -> Math.abs(val - Units.radiansToDegrees(-Math.PI / 4)) < 0.0001),
                doubleThat(val -> Math.abs(val - Units.radiansToDegrees(-Math.PI / 6)) < 0.0001),
                doubleThat(val -> Math.abs(val - Units.radiansToDegrees(-Math.PI / 3)) < 0.0001)), times(1));
    }

    @Test
    void testSetCameraPoseLargeValues() {
        Pose3d largePose = new Pose3d(
                new Translation3d(100.0, 200.0, 50.0),
                new Rotation3d(Math.PI, Math.PI / 2, 2 * Math.PI));

        limelight.setCameraPose(largePose);

        // Note: Rotation3d normalizes angles, so 2*PI becomes 0 and PI becomes 0 for roll
        mockHelpers.verify(() -> LimelightHelpers.setCameraPose_RobotSpace(
                eq(TEST_LIMELIGHT_NAME),
                eq(100.0),
                eq(200.0),
                eq(50.0),
                doubleThat(val -> Math.abs(val) < 0.0001), // Roll: PI normalizes to 0
                doubleThat(val -> Math.abs(val - 90.0) < 0.0001), // Pitch: PI/2 = 90 degrees
                doubleThat(val -> Math.abs(val - 180.0) < 0.0001) // Yaw: 2*PI normalizes to 180
        ), times(1));
    }

    //#endregion

    //#region Targeting Data Tests

    @Test
    void testGetHorizontalOffsetFromTarget() {
        double expectedDegrees = 15.5;
        mockHelpers.when(() -> LimelightHelpers.getTX(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedDegrees);

        Rotation2d result = limelight.getHorizontalOffsetFromTarget();

        assertEquals(expectedDegrees, result.getDegrees(), 0.001,
                "Horizontal offset should match mocked value");
    }

    @Test
    void testGetHorizontalOffsetFromTargetNegative() {
        double expectedDegrees = -20.3;
        mockHelpers.when(() -> LimelightHelpers.getTX(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedDegrees);

        Rotation2d result = limelight.getHorizontalOffsetFromTarget();

        assertEquals(expectedDegrees, result.getDegrees(), 0.001,
                "Horizontal offset should handle negative values");
    }

    @Test
    void testGetHorizontalOffsetFromTargetZero() {
        mockHelpers.when(() -> LimelightHelpers.getTX(TEST_LIMELIGHT_NAME))
                .thenReturn(0.0);

        Rotation2d result = limelight.getHorizontalOffsetFromTarget();

        assertEquals(0.0, result.getDegrees(), 0.001,
                "Horizontal offset should handle zero");
    }

    @Test
    void testGetVerticalOffsetFromTarget() {
        double expectedDegrees = 10.7;
        mockHelpers.when(() -> LimelightHelpers.getTY(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedDegrees);

        Rotation2d result = limelight.getVerticalOffsetFromTarget();

        assertEquals(expectedDegrees, result.getDegrees(), 0.001,
                "Vertical offset should match mocked value");
    }

    @Test
    void testGetVerticalOffsetFromTargetNegative() {
        double expectedDegrees = -15.2;
        mockHelpers.when(() -> LimelightHelpers.getTY(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedDegrees);

        Rotation2d result = limelight.getVerticalOffsetFromTarget();

        assertEquals(expectedDegrees, result.getDegrees(), 0.001,
                "Vertical offset should handle negative values");
    }

    @Test
    void testGetTargetArea() {
        double expectedArea = 25.5;
        mockHelpers.when(() -> LimelightHelpers.getTA(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedArea);

        double result = limelight.getTargetArea();

        assertEquals(expectedArea, result, 0.001,
                "Target area should match mocked value");
    }

    @Test
    void testGetTargetAreaZero() {
        mockHelpers.when(() -> LimelightHelpers.getTA(TEST_LIMELIGHT_NAME))
                .thenReturn(0.0);

        double result = limelight.getTargetArea();

        assertEquals(0.0, result, 0.001,
                "Target area should handle zero (no target)");
    }

    @Test
    void testGetTargetAreaMaximum() {
        mockHelpers.when(() -> LimelightHelpers.getTA(TEST_LIMELIGHT_NAME))
                .thenReturn(100.0);

        double result = limelight.getTargetArea();

        assertEquals(100.0, result, 0.001,
                "Target area should handle maximum value");
    }

    @Test
    void testGetPipelineLatencyMs() {
        double expectedLatency = 11.5;
        mockHelpers.when(() -> LimelightHelpers.getLatency_Pipeline(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedLatency);

        double result = limelight.getPipelineLatencyMs();

        assertEquals(expectedLatency, result, 0.001,
                "Pipeline latency should match mocked value");
    }

    @Test
    void testGetCapturePipelineLatencyMs() {
        double expectedLatency = 5.3;
        mockHelpers.when(() -> LimelightHelpers.getLatency_Capture(TEST_LIMELIGHT_NAME))
                .thenReturn(expectedLatency);

        double result = limelight.getCapturePipelineLatencyMs();

        assertEquals(expectedLatency, result, 0.001,
                "Capture latency should match mocked value");
    }

    //#endregion

    //#region Update Inputs Tests

    @Test
    void testUpdateInputsCallsHelpers() {
        LimelightInputs inputs = new LimelightInputs();

        mockHelpers.when(() -> LimelightHelpers.getTX(TEST_LIMELIGHT_NAME))
                .thenReturn(5.0);
        mockHelpers.when(() -> LimelightHelpers.getTY(TEST_LIMELIGHT_NAME))
                .thenReturn(3.0);

        limelight.updateInputs(inputs);

        // Verify that the helper methods were called
        mockHelpers.verify(() -> LimelightHelpers.getTX(TEST_LIMELIGHT_NAME), times(1));
        mockHelpers.verify(() -> LimelightHelpers.getTY(TEST_LIMELIGHT_NAME), times(1));
        mockHelpers.verify(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue(TEST_LIMELIGHT_NAME), times(1));
        mockHelpers.verify(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(TEST_LIMELIGHT_NAME), times(1));
        mockHelpers.verify(() -> LimelightHelpers.getLatestResults(TEST_LIMELIGHT_NAME), times(1));
    }

    //#endregion

    //#region Resource Management Tests

    @Test
    void testCloseDoesNotThrowException() {
        LimeLight ll = new LimeLight("test");
        assertDoesNotThrow(() -> ll.close(),
                "close() should not throw exception");
    }

    @Test
    void testMultipleCloseCallsDoNotThrowException() {
        LimeLight ll = new LimeLight("test");
        assertDoesNotThrow(() -> {
            ll.close();
            ll.close();
            ll.close();
        }, "Multiple close() calls should not throw exception");
    }

    //#endregion
}
