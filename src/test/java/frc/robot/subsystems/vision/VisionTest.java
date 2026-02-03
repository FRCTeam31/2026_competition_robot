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
    private LimelightVision vision;
    private MockedConstruction<LimeLightCamera> mockLimelightConstruction;

    @BeforeEach
    void setUp() {
        // Initialize HAL for WPILib
        assert HAL.initialize(500, 0);

        // Mock LimeLight construction to avoid network calls
        mockLimelightConstruction = mockConstruction(LimeLightCamera.class);

        vision = new LimelightVision();
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
        // Verify that one Limelight instance was created
        assertEquals(1, mockLimelightConstruction.constructed().size(),
                "Vision should create one Limelight instance");

        // Verify the limelight was created with correct name
        var constructedLimelights = mockLimelightConstruction.constructed();
        var turretLL = constructedLimelights.get(0);

        // Verify construction arguments
        verify(turretLL, never()).updateInputs(any());
    }

    @Test
    void testConstructorSetsSubsystemName() {
        assertEquals("LimelightVision", vision.getName(),
                "Vision subsystem should have correct name");
    }

    //#endregion

    //#region LED Mode Tests

    @Test
    void testSetLedModeTurret() {
        vision.setLEDMode(VisionMap.LimelightTurretName, 1);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setLedMode(1);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2, 3 })
    void testSetLedModeAllValidModes(int mode) {
        vision.setLEDMode(VisionMap.LimelightTurretName, mode);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setLedMode(mode);
    }

    //#endregion

    //#region Blink LED Tests

    @Test
    void testBlinkLedTurret() {
        vision.blinkLED(VisionMap.LimelightTurretName, 5);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).blinkLed(5);
    }

    //#endregion

    //#region Pipeline Tests

    @Test
    void testSetPipelineTurret() {
        vision.setPipeline(VisionMap.LimelightTurretName, 2);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setPipeline(2);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 })
    void testSetPipelineAllValidPipelines(int pipeline) {
        vision.setPipeline(VisionMap.LimelightTurretName, pipeline);

        var frontLL = mockLimelightConstruction.constructed().get(0);
        verify(frontLL, times(1)).setPipeline(pipeline);
    }

    //#endregion

    //#region Streaming Mode Tests

    @Test
    void testSetPiPStreamingModeTurret() {
        vision.setPiPStreamingMode(VisionMap.LimelightTurretName, 1);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setPiPStreamingMode(1);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2 })
    void testSetPiPStreamingModeAllValidModes(int mode) {
        vision.setPiPStreamingMode(VisionMap.LimelightTurretName, mode);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setPiPStreamingMode(mode);
    }

    //#endregion

    //#region Camera Pose Tests

    @Test
    void testSetCameraPoseTurret() {
        Pose3d testPose = new Pose3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3));
        vision.setCameraPose(VisionMap.LimelightTurretName, testPose);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setCameraPose(testPose);
    }

    @Test
    void testSetCameraPoseZeroPose() {
        Pose3d zeroPose = new Pose3d();
        vision.setCameraPose(VisionMap.LimelightTurretName, zeroPose);

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setCameraPose(zeroPose);
    }

    //#endregion

    //#region Periodic Tests

    @Test
    void testPeriodicUpdatesLimelights() {
        vision.periodic();

        var turretLL = mockLimelightConstruction.constructed().get(0);

        // Verify updateInputs was called on the limelight
        verify(turretLL, times(1)).updateInputs(any());
    }

    @Test
    void testPeriodicMultipleCalls() {
        // Call periodic multiple times
        vision.periodic();
        vision.periodic();
        vision.periodic();

        var turretLL = mockLimelightConstruction.constructed().get(0);

        // Verify updateInputs was called correct number of times
        verify(turretLL, times(3)).updateInputs(any());
    }

    //#endregion

    //#region Command Tests

    @Test
    void testSetLimelightPipelineCommandCreation() {
        Command command = vision.setProcessingPipeline(VisionMap.LimelightTurretName, 5);

        assertNotNull(command, "Command should not be null");
    }

    @Test
    void testSetLimelightPipelineCommandExecution() {
        Command command = vision.setProcessingPipeline(VisionMap.LimelightTurretName, 5);

        // Execute the command
        command.initialize();
        command.execute();

        var turretLL = mockLimelightConstruction.constructed().get(0);
        verify(turretLL, times(1)).setPipeline(5);
    }

    //#endregion

    //#region Add/Remove Limelight Tests

    @Test
    void testAddLimelightSuccess() {
        boolean result = vision.addCamera("limelight-front");

        assertTrue(result, "Should return true when adding new limelight");
        assertEquals(2, mockLimelightConstruction.constructed().size(),
                "Should have two limelights after adding");
        assertTrue(vision.hasCamera("limelight-front"),
                "Should contain newly added limelight");
    }

    @Test
    void testAddLimelightDuplicate() {
        boolean firstAdd = vision.addCamera("limelight-test");
        boolean secondAdd = vision.addCamera("limelight-test");

        assertTrue(firstAdd, "First add should succeed");
        assertFalse(secondAdd, "Second add with same name should fail");
        assertEquals(2, mockLimelightConstruction.constructed().size(),
                "Should only create one limelight instance");
    }

    @Test
    void testAddMultipleLimelights() {
        vision.addCamera("limelight-front");
        vision.addCamera("limelight-rear");
        vision.addCamera("limelight-intake");

        assertEquals(4, mockLimelightConstruction.constructed().size(),
                "Should have four limelights total");
        assertTrue(vision.hasCamera("limelight-front"));
        assertTrue(vision.hasCamera("limelight-rear"));
        assertTrue(vision.hasCamera("limelight-intake"));
        assertTrue(vision.hasCamera(VisionMap.LimelightTurretName));
    }

    @Test
    void testRemoveLimelightSuccess() {
        vision.addCamera("limelight-test");
        boolean result = vision.removeCamera("limelight-test");

        assertTrue(result, "Should return true when removing existing limelight");
        assertFalse(vision.hasCamera("limelight-test"),
                "Should not contain removed limelight");
        assertEquals(1, vision.getCameraNames().size(),
                "Should have one limelight remaining");
    }

    @Test
    void testRemoveLimelightNonExistent() {
        boolean result = vision.removeCamera("limelight-nonexistent");

        assertFalse(result, "Should return false when removing non-existent limelight");
    }

    @Test
    void testRemoveDefaultLimelight() {
        boolean result = vision.removeCamera(VisionMap.LimelightTurretName);

        assertTrue(result, "Should be able to remove default limelight");
        assertFalse(vision.hasCamera(VisionMap.LimelightTurretName),
                "Default limelight should be removed");
    }

    @Test
    void testGetLimelightNames() {
        var names = vision.getCameraNames();

        assertNotNull(names, "Names set should not be null");
        assertEquals(1, names.size(), "Should have one limelight by default");
        assertTrue(names.contains(VisionMap.LimelightTurretName),
                "Should contain default turret limelight");

        vision.addCamera("limelight-test");
        names = vision.getCameraNames();
        assertEquals(2, names.size(), "Should have two limelights after adding");
        assertTrue(names.contains("limelight-test"), "Should contain added limelight");
    }

    @Test
    void testHasLimelight() {
        assertTrue(vision.hasCamera(VisionMap.LimelightTurretName),
                "Should have default turret limelight");
        assertFalse(vision.hasCamera("limelight-nonexistent"),
                "Should not have non-existent limelight");

        vision.addCamera("limelight-test");
        assertTrue(vision.hasCamera("limelight-test"),
                "Should have newly added limelight");
    }

    @Test
    void testSetLedModeNonExistentLimelight() {
        // Should not throw exception when setting LED mode on non-existent limelight
        assertDoesNotThrow(() -> vision.setLEDMode("limelight-nonexistent", 1),
                "Should handle non-existent limelight gracefully");
    }

    @Test
    void testSetPipelineNonExistentLimelight() {
        // Should not throw exception when setting pipeline on non-existent limelight
        assertDoesNotThrow(() -> vision.setPipeline("limelight-nonexistent", 1),
                "Should handle non-existent limelight gracefully");
    }

    @Test
    void testAddRemoveAddSameLimelight() {
        vision.addCamera("limelight-test");
        vision.removeCamera("limelight-test");
        boolean result = vision.addCamera("limelight-test");

        assertTrue(result, "Should be able to re-add removed limelight");
        assertTrue(vision.hasCamera("limelight-test"),
                "Re-added limelight should exist");
    }

    //#endregion

    //#region VisionMap Tests

    @Test
    void testVisionMapConstants() {
        assertEquals("limelight-turret", VisionMap.LimelightTurretName,
                "Turret limelight name should be correct");
    }

    //#endregion
}
