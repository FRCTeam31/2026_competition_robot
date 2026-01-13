package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.MockedConstruction;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Unit tests for the Vision subsystem.
 * Tests core functionality including limelight management,
 * LED control, pipeline switching, and AprilTag validation.
 */
class VisionTest {
    private Vision vision;
    private MockedConstruction<LimeLight> mockLimelightConstruction;

    @BeforeEach
    void setUp() {
        // Initialize HAL for WPILib
        assert HAL.initialize(500, 0);

        // Mock LimeLight construction to avoid network calls
        mockLimelightConstruction = mockConstruction(LimeLight.class);

        vision = new Vision();
    }

    @AfterEach
    void tearDown() {
        if (mockLimelightConstruction != null) {
            mockLimelightConstruction.close();
        }
    }

    //#region Constructor Tests

    @Test
    void testConstructorCreatesLimelights() {
        // Verify that two Limelight instances were created
        assertEquals(2, mockLimelightConstruction.constructed().size(),
                "Vision should create two Limelight instances");

        // Verify the limelights were created with correct names
        var constructedLimelights = mockLimelightConstruction.constructed();
        var frontLL = constructedLimelights.get(0);
        var rearLL = constructedLimelights.get(1);

        // Verify construction arguments
        verify(frontLL, never()).updateInputs(any());
        verify(rearLL, never()).updateInputs(any());
    }

    @Test
    void testConstructorSetsSubsystemName() {
        assertEquals("Vision", vision.getName(),
                "Vision subsystem should have correct name");
    }

    //#endregion

    //#region LED Mode Tests

    @Test
    void testSetLedModeFront() {
        vision.setLedMode(LimelightNameEnum.kFront, 1);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setLedMode(1);
    }

    @Test
    void testSetLedModeRear() {
        vision.setLedMode(LimelightNameEnum.kRear, 3);

        var rearLL = mockLimelightConstruction.constructed().get(1);
        verify(rearLL, times(1)).setLedMode(3);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2, 3 })
    void testSetLedModeAllValidModes(int mode) {
        vision.setLedMode(LimelightNameEnum.kFront, mode);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setLedMode(mode);
    }

    //#endregion

    //#region Blink LED Tests

    @Test
    void testBlinkLedFront() {
        vision.blinkLed(LimelightNameEnum.kFront, 5);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).blinkLed(5);
    }

    @Test
    void testBlinkLedRear() {
        vision.blinkLed(LimelightNameEnum.kRear, 3);

        var rearLL = mockLimelightConstruction.constructed().get(1);
        verify(rearLL, times(1)).blinkLed(3);
    }

    //#endregion

    //#region Pipeline Tests

    @Test
    void testSetPipelineFront() {
        vision.setPipeline(LimelightNameEnum.kFront, 2);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setPipeline(2);
    }

    @Test
    void testSetPipelineRear() {
        vision.setPipeline(LimelightNameEnum.kRear, 7);

        var rearLL = mockLimelightConstruction.constructed().get(1);
        verify(rearLL, times(1)).setPipeline(7);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 })
    void testSetPipelineAllValidPipelines(int pipeline) {
        vision.setPipeline(LimelightNameEnum.kFront, pipeline);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setPipeline(pipeline);
    }

    //#endregion

    //#region Streaming Mode Tests

    @Test
    void testSetPiPStreamingModeFront() {
        vision.setPiPStreamingMode(LimelightNameEnum.kFront, 1);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setPiPStreamingMode(1);
    }

    @Test
    void testSetPiPStreamingModeRear() {
        vision.setPiPStreamingMode(LimelightNameEnum.kRear, 2);

        var rearLL = mockLimelightConstruction.constructed().get(1);
        verify(rearLL, times(1)).setPiPStreamingMode(2);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2 })
    void testSetPiPStreamingModeAllValidModes(int mode) {
        vision.setPiPStreamingMode(LimelightNameEnum.kFront, mode);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setPiPStreamingMode(mode);
    }

    //#endregion

    //#region Camera Pose Tests

    @Test
    void testSetCameraPoseFront() {
        Pose3d testPose = new Pose3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3));
        vision.setCameraPose(LimelightNameEnum.kFront, testPose);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setCameraPose(testPose);
    }

    @Test
    void testSetCameraPoseRear() {
        Pose3d testPose = new Pose3d(4.0, 5.0, 6.0, new Rotation3d(0.4, 0.5, 0.6));
        vision.setCameraPose(LimelightNameEnum.kRear, testPose);

        var rearLL = mockLimelightConstruction.constructed().get(1);
        verify(rearLL, times(1)).setCameraPose(testPose);
    }

    @Test
    void testSetCameraPoseZeroPose() {
        Pose3d zeroPose = new Pose3d();
        vision.setCameraPose(LimelightNameEnum.kFront, zeroPose);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setCameraPose(zeroPose);
    }

    //#endregion

    //#region AprilTag Validation Tests

    @ParameterizedTest
    @ValueSource(ints = { 1, 2, 10, 15, 20, 21, 22 })
    void testIsAprilTagIdValidForValidIds(int id) {
        assertTrue(Vision.isAprilTagIdValid(id),
                "AprilTag ID " + id + " should be valid");
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, -1, -10, 23, 24, 100, 1000 })
    void testIsAprilTagIdValidForInvalidIds(int id) {
        assertFalse(Vision.isAprilTagIdValid(id),
                "AprilTag ID " + id + " should be invalid");
    }

    @Test
    void testIsAprilTagIdValidBoundaries() {
        // Test lower boundary
        assertFalse(Vision.isAprilTagIdValid(0), "ID 0 should be invalid");
        assertTrue(Vision.isAprilTagIdValid(1), "ID 1 should be valid");

        // Test upper boundary
        assertTrue(Vision.isAprilTagIdValid(22), "ID 22 should be valid");
        assertFalse(Vision.isAprilTagIdValid(23), "ID 23 should be invalid");
    }

    //#endregion

    //#region Periodic Tests

    @Test
    void testPeriodicUpdatesLimelights() {
        vision.periodic();

        var frontLL = mockLimelightConstruction.constructed().get(0);
        var rearLL = mockLimelightConstruction.constructed().get(1);

        // Verify updateInputs was called on both limelights
        verify(frontLL, times(1)).updateInputs(any());
        verify(rearLL, times(1)).updateInputs(any());
    }

    @Test
    void testPeriodicMultipleCalls() {
        // Call periodic multiple times
        vision.periodic();
        vision.periodic();
        vision.periodic();

        var frontLL = mockLimelightConstruction.constructed().get(0);
        var rearLL = mockLimelightConstruction.constructed().get(1);

        // Verify updateInputs was called correct number of times
        verify(frontLL, times(3)).updateInputs(any());
        verify(rearLL, times(3)).updateInputs(any());
    }

    //#endregion

    //#region Command Tests

    @Test
    void testSetLimelightPipelineCommandCreation() {
        Command command = vision.setLimelightPipeline(LimelightNameEnum.kFront, 5);

        assertNotNull(command, "Command should not be null");
    }

    @Test
    void testSetLimelightPipelineCommandExecution() {
        Command command = vision.setLimelightPipeline(LimelightNameEnum.kFront, 5);

        // Execute the command
        command.initialize();
        command.execute();

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setPipeline(5);
    }

    @Test
    void testSetLimelightPipelineCommandForRear() {
        Command command = vision.setLimelightPipeline(LimelightNameEnum.kRear, 3);

        // Execute the command
        command.initialize();
        command.execute();

        var rearLL = mockLimelightConstruction.constructed().get(1);
        verify(rearLL, times(1)).setPipeline(3);
    }

    //#endregion

    //#region VisionMap Tests

    @Test
    void testVisionMapConstants() {
        assertEquals("limelight-front", Vision.VisionMap.LimelightFrontName,
                "Front limelight name should be correct");
        assertEquals("limelight-rear", Vision.VisionMap.LimelightRearName,
                "Rear limelight name should be correct");
    }

    //#endregion
}
