package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.photon.PhotonCameraInputsAutoLogged;
import frc.robot.subsystems.vision.photon.PhotonVisionCamera;

/**
 * Unit tests for the PhotonVisionCamera class.
 * Tests construction, input updates, pipeline control,
 * camera transforms, and standard deviation estimation heuristics.
 */
class PhotonVisionCameraTest {
    private PhotonVisionCamera camera;
    private PhotonCameraInputsAutoLogged inputs;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        camera = new PhotonVisionCamera("test-camera", new Transform3d());
        inputs = new PhotonCameraInputsAutoLogged();
    }

    //#region Constructor Tests

    @Test
    void testConstructor_InitializesSuccessfully() {
        assertNotNull(camera, "Camera should be created successfully");
    }

    @Test
    void testConstructor_InitializesWithNamedCamera() {
        var namedCamera = new PhotonVisionCamera("photon-front", new Transform3d());
        assertNotNull(namedCamera, "Named camera should be created successfully");
    }

    @Test
    void testConstructor_InitializesWithTransform() {
        Transform3d transform = new Transform3d(
                new Translation3d(0.3, 0.1, 0.5),
                new Rotation3d(0, Math.toRadians(-15), 0));
        var transformedCamera = new PhotonVisionCamera("photon-offset", transform);
        assertNotNull(transformedCamera, "Camera with transform should be created successfully");
    }

    @Test
    void testConstructor_InitializesSimInSimMode() {
        // In test environment, Robot.isSimulation() returns true
        assertNotNull(camera.Sim, "Sim should be initialized in simulation mode");
    }

    @Test
    void testConstructor_AcceptsIdentityTransform() {
        var identityCamera = new PhotonVisionCamera("identity-cam", new Transform3d());
        assertNotNull(identityCamera, "Camera with identity transform should be created");
    }

    @Test
    void testConstructor_AcceptsLargeOffsetTransform() {
        Transform3d largeTransform = new Transform3d(
                new Translation3d(2.0, 1.5, 1.0),
                new Rotation3d(Math.PI / 4, Math.PI / 6, Math.PI / 3));
        var largeOffsetCamera = new PhotonVisionCamera("far-cam", largeTransform);
        assertNotNull(largeOffsetCamera, "Camera with large offset should be created");
    }

    //#endregion

    //#region updateInputs - Empty Results Tests

    @Test
    void testUpdateInputs_NoResults_SetsLatestResultNull() {
        camera.updateInputs(inputs);
        assertNull(inputs.LatestResult, "LatestResult should be null when no results");
    }

    @Test
    void testUpdateInputs_NoResults_SetsTargetCountZero() {
        camera.updateInputs(inputs);
        assertEquals(0, inputs.TargetCount, "TargetCount should be 0 when no results");
    }

    @Test
    void testUpdateInputs_NoResults_SetsPrimaryTargetIdNegativeOne() {
        camera.updateInputs(inputs);
        assertEquals(-1, inputs.PrimaryTargetId, "PrimaryTargetId should be -1 when no results");
    }

    @Test
    void testUpdateInputs_NoResults_SetsPrimaryTargetRotationToZero() {
        camera.updateInputs(inputs);
        assertEquals(new Rotation3d(), inputs.PrimaryTargetRotation2d,
                "PrimaryTargetRotation2d should be zero rotation when no results");
    }

    @Test
    void testUpdateInputs_NoResults_SetsBotPoseEstimateNull() {
        camera.updateInputs(inputs);
        assertNull(inputs.BotPoseEstimate, "BotPoseEstimate should be null when no results");
    }

    @Test
    void testUpdateInputs_NoResults_SetsTimestampNegativeOne() {
        camera.updateInputs(inputs);
        assertEquals(-1, inputs.TimestampSeconds, "TimestampSeconds should be -1 when no results");
    }

    @Test
    void testUpdateInputs_MultipleCallsWithNoResults_RemainsConsistent() {
        // First call
        camera.updateInputs(inputs);
        assertNull(inputs.LatestResult);
        assertEquals(0, inputs.TargetCount);
        assertEquals(-1, inputs.PrimaryTargetId);

        // Second call - should not NPE and should remain consistent
        camera.updateInputs(inputs);
        assertNull(inputs.LatestResult);
        assertEquals(0, inputs.TargetCount);
        assertEquals(-1, inputs.PrimaryTargetId);

        // Third call
        camera.updateInputs(inputs);
        assertNull(inputs.LatestResult);
        assertEquals(0, inputs.TargetCount);
    }

    @Test
    void testUpdateInputs_NoResults_DoesNotModifyStdDevs() {
        var originalStdDevs = inputs.CurrentStdDevs;
        camera.updateInputs(inputs);
        // When there are no results, updateEstimationStdDevs is not called,
        // so stdDevs should remain at their default value
        assertEquals(originalStdDevs, inputs.CurrentStdDevs,
                "StdDevs should not change when there are no results");
    }

    //#endregion

    //#region updateInputs - Default Input Values Tests

    @Test
    void testInputs_DefaultLatestResult_IsNotNull() {
        // PhotonCameraInputs initializes LatestResult to a new PhotonPipelineResult
        assertNotNull(inputs.LatestResult, "Default LatestResult should not be null");
    }

    @Test
    void testInputs_DefaultTargetCount_IsZero() {
        assertEquals(0, inputs.TargetCount, "Default TargetCount should be 0");
    }

    @Test
    void testInputs_DefaultPrimaryTargetId_IsNegativeOne() {
        assertEquals(-1, inputs.PrimaryTargetId, "Default PrimaryTargetId should be -1");
    }

    @Test
    void testInputs_DefaultBotPoseEstimate_IsDefaultPose() {
        assertEquals(new Pose2d(), inputs.BotPoseEstimate, "Default BotPoseEstimate should be default Pose2d");
    }

    @Test
    void testInputs_DefaultStdDevs_IsSingleTagStdDevs() {
        assertEquals(VisionMap.kSingleTagStdDevs, inputs.CurrentStdDevs,
                "Default StdDevs should be single tag std devs");
    }

    @Test
    void testInputs_DefaultTimestamp_IsZero() {
        assertEquals(0.0, inputs.TimestampSeconds, "Default TimestampSeconds should be 0.0");
    }

    @Test
    void testInputs_DefaultPrimaryTargetRotation_IsZero() {
        assertEquals(new Rotation3d(), inputs.PrimaryTargetRotation2d,
                "Default PrimaryTargetRotation2d should be zero rotation");
    }

    //#endregion

    //#region setPipeline Tests

    @Test
    void testSetPipeline_DoesNotThrow() {
        assertDoesNotThrow(() -> camera.setPipeline(0),
                "setPipeline should not throw");
    }

    @Test
    void testSetPipeline_AcceptsZero() {
        assertDoesNotThrow(() -> camera.setPipeline(0),
                "setPipeline should accept pipeline 0");
    }

    @Test
    void testSetPipeline_AcceptsPositiveIndex() {
        assertDoesNotThrow(() -> camera.setPipeline(5),
                "setPipeline should accept positive pipeline index");
    }

    @Test
    void testSetPipeline_AcceptsHighIndex() {
        assertDoesNotThrow(() -> camera.setPipeline(9),
                "setPipeline should accept high pipeline index");
    }

    @Test
    void testSetPipeline_MultipleCalls() {
        assertDoesNotThrow(() -> {
            camera.setPipeline(0);
            camera.setPipeline(3);
            camera.setPipeline(7);
        }, "setPipeline should handle multiple consecutive calls");
    }

    //#endregion

    //#region setRobotCameraTransform Tests

    @Test
    void testSetRobotCameraTransform_IdentityTransform() {
        assertDoesNotThrow(() -> camera.setRobotCameraTransform(new Transform3d()),
                "Should accept identity transform");
    }

    @Test
    void testSetRobotCameraTransform_WithTranslation() {
        Transform3d transform = new Transform3d(
                new Translation3d(0.5, 0.2, 0.3),
                new Rotation3d());
        assertDoesNotThrow(() -> camera.setRobotCameraTransform(transform),
                "Should accept transform with translation");
    }

    @Test
    void testSetRobotCameraTransform_WithRotation() {
        Transform3d transform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, Math.toRadians(-30), Math.toRadians(15)));
        assertDoesNotThrow(() -> camera.setRobotCameraTransform(transform),
                "Should accept transform with rotation");
    }

    @Test
    void testSetRobotCameraTransform_FullTransform() {
        Transform3d transform = new Transform3d(
                new Translation3d(0.3, -0.1, 0.5),
                new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180)));
        assertDoesNotThrow(() -> camera.setRobotCameraTransform(transform),
                "Should accept full transform with translation and rotation");
    }

    @Test
    void testSetRobotCameraTransform_MultipleCalls() {
        Transform3d transform1 = new Transform3d(
                new Translation3d(0.1, 0, 0.3), new Rotation3d());
        Transform3d transform2 = new Transform3d(
                new Translation3d(0.5, 0.2, 0.8), new Rotation3d(0, Math.toRadians(-10), 0));
        assertDoesNotThrow(() -> {
            camera.setRobotCameraTransform(transform1);
            camera.setRobotCameraTransform(transform2);
        }, "Should handle multiple transform updates");
    }

    //#endregion

    //#region Sim Field Tests

    @Test
    void testSim_IsNotNullInSimulation() {
        assertNotNull(camera.Sim, "Sim field should be non-null in simulation");
    }

    @Test
    void testSim_AllCamerasGetUniqueSims() {
        var camera2 = new PhotonVisionCamera("test-camera-2", new Transform3d());
        assertNotNull(camera.Sim);
        assertNotNull(camera2.Sim);
        assertNotSame(camera.Sim, camera2.Sim,
                "Different cameras should have different sim instances");
    }

    //#endregion

    //#region StdDevs Constants Tests

    @Test
    void testVisionMap_SingleTagStdDevs_HasExpectedValues() {
        assertEquals(VecBuilder.fill(4, 4, 8), VisionMap.kSingleTagStdDevs,
                "Single tag std devs should match expected values");
    }

    @Test
    void testVisionMap_MultiTagStdDevs_HasExpectedValues() {
        assertEquals(VecBuilder.fill(0.5, 0.5, 1), VisionMap.kMultiTagStdDevs,
                "Multi tag std devs should match expected values");
    }

    @Test
    void testVisionMap_MultiTagStdDevs_AreLessThanSingleTag() {
        for (int i = 0; i < 3; i++) {
            assertTrue(VisionMap.kMultiTagStdDevs.get(i, 0) < VisionMap.kSingleTagStdDevs.get(i, 0),
                    "Multi tag std devs should be less than single tag std devs at index " + i);
        }
    }

    //#endregion

    //#region Integration Tests

    @Test
    void testFullCycle_ConstructUpdateSetPipeline() {
        // Construct
        var testCam = new PhotonVisionCamera("cycle-cam",
                new Transform3d(new Translation3d(0.2, 0, 0.4), new Rotation3d()));

        // Update inputs (will get empty results in test)
        var testInputs = new PhotonCameraInputsAutoLogged();
        testCam.updateInputs(testInputs);

        // Verify empty-result state
        assertNull(testInputs.LatestResult);
        assertEquals(-1, testInputs.PrimaryTargetId);

        // Set pipeline
        assertDoesNotThrow(() -> testCam.setPipeline(2));

        // Update transform
        assertDoesNotThrow(() -> testCam.setRobotCameraTransform(
                new Transform3d(new Translation3d(0.25, 0, 0.45), new Rotation3d())));

        // Update inputs again
        testCam.updateInputs(testInputs);
        assertNull(testInputs.LatestResult);
    }

    @Test
    void testMultipleCameras_IndependentInputs() {
        var cam1 = new PhotonVisionCamera("cam-1", new Transform3d());
        var cam2 = new PhotonVisionCamera("cam-2", new Transform3d(
                new Translation3d(0.5, 0, 0), new Rotation3d()));

        var inputs1 = new PhotonCameraInputsAutoLogged();
        var inputs2 = new PhotonCameraInputsAutoLogged();

        cam1.updateInputs(inputs1);
        cam2.updateInputs(inputs2);

        // Both should have empty results independently
        assertNull(inputs1.LatestResult);
        assertNull(inputs2.LatestResult);
        assertEquals(-1, inputs1.PrimaryTargetId);
        assertEquals(-1, inputs2.PrimaryTargetId);
    }

    @Test
    void testUpdateInputs_AfterPipelineChange_DoesNotThrow() {
        camera.setPipeline(3);
        assertDoesNotThrow(() -> camera.updateInputs(inputs),
                "updateInputs should work after pipeline change");
    }

    @Test
    void testUpdateInputs_AfterTransformChange_DoesNotThrow() {
        camera.setRobotCameraTransform(new Transform3d(
                new Translation3d(1, 0, 0.5), new Rotation3d()));
        assertDoesNotThrow(() -> camera.updateInputs(inputs),
                "updateInputs should work after transform change");
    }

    //#endregion
}
