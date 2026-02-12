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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.subsystems.vision.photon.PhotonVision;
import frc.robot.subsystems.vision.photon.PhotonVisionCamera;

/**
 * Unit tests for the PhotonVision subsystem.
 * Tests core functionality including camera management,
 * pipeline switching, pose estimation, and camera transforms.
 */
class PhotonVisionTest {
    private PhotonVision vision;
    private MockedConstruction<PhotonVisionCamera> mockCameraConstruction;

    @BeforeEach
    void setUp() {
        // Initialize HAL for WPILib
        assert HAL.initialize(500, 0);

        // Clear any existing photon vision data from SuperStructure
        SuperStructure.VisionPhotons.clear();

        // Mock PhotonVisionCamera construction to avoid network calls
        mockCameraConstruction = mockConstruction(PhotonVisionCamera.class);

        vision = new PhotonVision();
    }

    @AfterEach
    void tearDown() {
        if (mockCameraConstruction != null) {
            mockCameraConstruction.close();
        }
        SuperStructure.VisionPhotons.clear();
    }

    //#region Constructor Tests

    @Test
    void testConstructorSetsSubsystemName() {
        assertEquals("PhotonVision", vision.getName(),
                "PhotonVision subsystem should have correct name");
    }

    @Test
    void testConstructorStartsWithNoCameras() {
        // PhotonVision doesn't add default cameras in constructor
        assertEquals(0, vision.getCameraNames().size(),
                "PhotonVision should start with no cameras by default");
    }

    //#endregion

    //#region Add Camera Tests

    @Test
    void testAddCameraSuccess() {
        Transform3d transform = new Transform3d(
                new Translation3d(0.1, 0.2, 0.3),
                new Rotation3d(0, 0, 0));
        boolean result = vision.addCamera("photon-front", transform);

        assertTrue(result, "Should return true when adding new camera");
        assertEquals(1, mockCameraConstruction.constructed().size(),
                "Should have one camera after adding");
        assertTrue(vision.hasCamera("photon-front"),
                "Should contain newly added camera");
    }

    @Test
    void testAddCameraDuplicate() {
        Transform3d transform = new Transform3d();
        boolean firstAdd = vision.addCamera("photon-test", transform);
        boolean secondAdd = vision.addCamera("photon-test", transform);

        assertTrue(firstAdd, "First add should succeed");
        assertFalse(secondAdd, "Second add with same name should fail");
        assertEquals(1, mockCameraConstruction.constructed().size(),
                "Should only create one camera instance");
    }

    @Test
    void testAddMultipleCameras() {
        Transform3d transform = new Transform3d();
        vision.addCamera("photon-front", transform);
        vision.addCamera("photon-rear", transform);
        vision.addCamera("photon-turret", transform);

        assertEquals(3, mockCameraConstruction.constructed().size(),
                "Should have three cameras total");
        assertTrue(vision.hasCamera("photon-front"));
        assertTrue(vision.hasCamera("photon-rear"));
        assertTrue(vision.hasCamera("photon-turret"));
    }

    @Test
    void testAddCameraWithDifferentTransforms() {
        Transform3d frontTransform = new Transform3d(
                new Translation3d(0.3, 0, 0.5),
                new Rotation3d(0, Math.toRadians(-15), 0));
        Transform3d rearTransform = new Transform3d(
                new Translation3d(-0.3, 0, 0.5),
                new Rotation3d(0, Math.toRadians(-15), Math.PI));

        vision.addCamera("photon-front", frontTransform);
        vision.addCamera("photon-rear", rearTransform);

        assertEquals(2, vision.getCameraNames().size(),
                "Should have two cameras with different transforms");
    }

    @Test
    void testAddCameraUpdatesSuperStructure() {
        Transform3d transform = new Transform3d();
        vision.addCamera("photon-test", transform);

        assertTrue(SuperStructure.VisionPhotons.containsKey("photon-test"),
                "SuperStructure should contain camera inputs after adding");
    }

    //#endregion

    //#region Remove Camera Tests

    @Test
    void testRemoveCameraSuccess() {
        vision.addCamera("photon-test", new Transform3d());
        boolean result = vision.removeCamera("photon-test");

        assertTrue(result, "Should return true when removing existing camera");
        assertFalse(vision.hasCamera("photon-test"),
                "Should not contain removed camera");
    }

    @Test
    void testRemoveCameraNonExistent() {
        boolean result = vision.removeCamera("photon-nonexistent");

        assertFalse(result, "Should return false when removing non-existent camera");
    }

    @Test
    void testRemoveCameraUpdatesSuperStructure() {
        vision.addCamera("photon-test", new Transform3d());
        vision.removeCamera("photon-test");

        assertFalse(SuperStructure.VisionPhotons.containsKey("photon-test"),
                "SuperStructure should not contain camera inputs after removing");
    }

    @Test
    void testAddRemoveAddSameCamera() {
        Transform3d transform = new Transform3d();
        vision.addCamera("photon-test", transform);
        vision.removeCamera("photon-test");
        boolean result = vision.addCamera("photon-test", transform);

        assertTrue(result, "Should be able to re-add removed camera");
        assertTrue(vision.hasCamera("photon-test"),
                "Re-added camera should exist");
    }

    //#endregion

    //#region Get Camera Names Tests

    @Test
    void testGetCameraNamesEmpty() {
        var names = vision.getCameraNames();

        assertNotNull(names, "Names set should not be null");
        assertEquals(0, names.size(), "Should have no cameras by default");
    }

    @Test
    void testGetCameraNamesAfterAdding() {
        vision.addCamera("photon-front", new Transform3d());
        vision.addCamera("photon-rear", new Transform3d());

        var names = vision.getCameraNames();

        assertEquals(2, names.size(), "Should have two cameras after adding");
        assertTrue(names.contains("photon-front"), "Should contain front camera");
        assertTrue(names.contains("photon-rear"), "Should contain rear camera");
    }

    //#endregion

    //#region Has Camera Tests

    @Test
    void testHasCameraFalseWhenEmpty() {
        assertFalse(vision.hasCamera("photon-any"),
                "Should not have any camera when empty");
    }

    @Test
    void testHasCameraAfterAdding() {
        vision.addCamera("photon-test", new Transform3d());

        assertTrue(vision.hasCamera("photon-test"),
                "Should have camera after adding");
        assertFalse(vision.hasCamera("photon-other"),
                "Should not have camera that wasn't added");
    }

    //#endregion

    //#region Pipeline Tests

    @Test
    void testSetPipelineExistingCamera() {
        vision.addCamera("photon-test", new Transform3d());
        vision.setPipeline("photon-test", 2);

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(1)).setPipeline(2);
    }

    @ParameterizedTest
    @ValueSource(ints = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 })
    void testSetPipelineAllValidPipelines(int pipeline) {
        vision.addCamera("photon-test", new Transform3d());
        vision.setPipeline("photon-test", pipeline);

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(1)).setPipeline(pipeline);
    }

    @Test
    void testSetPipelineNonExistentCamera() {
        // Should not throw exception when setting pipeline on non-existent camera
        assertDoesNotThrow(() -> vision.setPipeline("photon-nonexistent", 1),
                "Should handle non-existent camera gracefully");
    }

    @Test
    void testSetPipelineMultipleCameras() {
        vision.addCamera("photon-front", new Transform3d());
        vision.addCamera("photon-rear", new Transform3d());

        vision.setPipeline("photon-front", 1);
        vision.setPipeline("photon-rear", 2);

        var frontCamera = mockCameraConstruction.constructed().get(0);
        var rearCamera = mockCameraConstruction.constructed().get(1);

        verify(frontCamera, times(1)).setPipeline(1);
        verify(rearCamera, times(1)).setPipeline(2);
    }

    //#endregion

    //#region Camera Pose Tests

    @Test
    void testSetCameraPoseExistingCamera() {
        vision.addCamera("photon-test", new Transform3d());
        Pose3d testPose = new Pose3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3));

        vision.setCameraPose("photon-test", testPose);

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(1)).setRobotCameraTransform(any(Transform3d.class));
    }

    @Test
    void testSetCameraPoseZeroPose() {
        vision.addCamera("photon-test", new Transform3d());
        Pose3d zeroPose = Pose3d.kZero;

        vision.setCameraPose("photon-test", zeroPose);

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(1)).setRobotCameraTransform(any(Transform3d.class));
    }

    @Test
    void testSetCameraPoseNonExistentCamera() {
        Pose3d testPose = new Pose3d(1.0, 2.0, 3.0, new Rotation3d());

        // Should not throw exception
        assertDoesNotThrow(() -> vision.setCameraPose("photon-nonexistent", testPose),
                "Should handle non-existent camera gracefully");
    }

    //#endregion

    //#region Periodic Tests

    @Test
    void testPeriodicUpdatesCameras() {
        vision.addCamera("photon-test", new Transform3d());
        vision.periodic();

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(1)).updateInputs(any());
    }

    @Test
    void testPeriodicUpdatesAllCameras() {
        vision.addCamera("photon-front", new Transform3d());
        vision.addCamera("photon-rear", new Transform3d());

        vision.periodic();

        var frontCamera = mockCameraConstruction.constructed().get(0);
        var rearCamera = mockCameraConstruction.constructed().get(1);

        verify(frontCamera, times(1)).updateInputs(any());
        verify(rearCamera, times(1)).updateInputs(any());
    }

    @Test
    void testPeriodicMultipleCalls() {
        vision.addCamera("photon-test", new Transform3d());

        vision.periodic();
        vision.periodic();
        vision.periodic();

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(3)).updateInputs(any());
    }

    @Test
    void testPeriodicWithNoCameras() {
        // Should not throw when no cameras are added
        assertDoesNotThrow(() -> vision.periodic(),
                "Periodic should handle empty camera list");
    }

    //#endregion

    //#region Command Tests

    @Test
    void testSetProcessingPipelineCommandCreation() {
        vision.addCamera("photon-test", new Transform3d());
        Command command = vision.setProcessingPipeline("photon-test", 5);

        assertNotNull(command, "Command should not be null");
    }

    @Test
    void testSetProcessingPipelineCommandExecution() {
        vision.addCamera("photon-test", new Transform3d());
        Command command = vision.setProcessingPipeline("photon-test", 5);

        // Execute the command
        command.initialize();
        command.execute();

        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera, times(1)).setPipeline(5);
    }

    @Test
    void testSetProcessingPipelineCommandNonExistentCamera() {
        Command command = vision.setProcessingPipeline("photon-nonexistent", 5);

        // Should not throw when executing command for non-existent camera
        assertDoesNotThrow(() -> {
            command.initialize();
            command.execute();
        }, "Command should handle non-existent camera gracefully");
    }

    //#endregion

    //#region Integration Tests

    @Test
    void testFullWorkflow() {
        // Add camera
        Transform3d transform = new Transform3d(
                new Translation3d(0.2, 0, 0.4),
                new Rotation3d(0, Math.toRadians(-10), 0));
        vision.addCamera("photon-turret", transform);

        assertTrue(vision.hasCamera("photon-turret"));
        assertTrue(SuperStructure.VisionPhotons.containsKey("photon-turret"));

        // Set pipeline
        vision.setPipeline("photon-turret", 1);
        var camera = mockCameraConstruction.constructed().get(0);
        verify(camera).setPipeline(1);

        // Update pose
        Pose3d newPose = new Pose3d(0.25, 0, 0.45, new Rotation3d(0, Math.toRadians(-12), 0));
        vision.setCameraPose("photon-turret", newPose);
        verify(camera).setRobotCameraTransform(any(Transform3d.class));

        // Run periodic
        vision.periodic();
        verify(camera).updateInputs(any());

        // Remove camera
        vision.removeCamera("photon-turret");
        assertFalse(vision.hasCamera("photon-turret"));
        assertFalse(SuperStructure.VisionPhotons.containsKey("photon-turret"));
    }

    //#endregion
}
