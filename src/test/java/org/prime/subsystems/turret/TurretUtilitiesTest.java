// package org.prime.subsystems.turret;

// import static org.junit.jupiter.api.Assertions.*;

// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Nested;
// import org.junit.jupiter.api.Test;
// import org.prime.util.MutVector;
// import org.prime.util.PhysicsConstants;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;

// /**
//  * Tests for the whitelabel TurretUtilities math functions.
//  * These test pure geometry and projectile physics with no subsystem dependencies.
//  */
// class TurretUtilitiesTest {

//     private static final double EPSILON = 1e-4;
//     private static final double ANGLE_EPSILON = 0.5; // degrees

//     // Reasonable defaults for a turret - tests are not tied to any specific robot's TurretMap
//     private static final double SHOOTER_HEIGHT = 0.45;
//     private static final double OVERSHOOT_HEIGHT = 0.2;

//     @BeforeEach
//     void setUp() {
//         assert HAL.initialize(500, 0);
//     }

//     // =========================================================================
//     // calculateAimVector tests
//     // =========================================================================

//     @Nested
//     class CalculateAimVectorTests {

//         /**
//          * A target directly in front of the source (+X direction) should yield a yaw near 0-degrees.
//          */
//         @Test
//         void yawIsZero_WhenTargetIsDirectlyAhead() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(5, 0, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertEquals(0.0, result.getYaw(), ANGLE_EPSILON,
//                     "Yaw should be ~0-degrees when target is directly in +X direction");
//         }

//         /**
//          * A target directly to the left (+Y direction) should yield a yaw near 90-degrees.
//          */
//         @Test
//         void yawIs90_WhenTargetIsDirectlyLeft() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(0, 5, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertEquals(90.0, result.getYaw(), ANGLE_EPSILON,
//                     "Yaw should be ~90-degrees when target is directly in +Y direction");
//         }

//         /**
//          * A target behind the source (-X direction) should yield yaw near +-180-degrees.
//          */
//         @Test
//         void yawIs180_WhenTargetIsBehind() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(-5, 0, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertEquals(180.0, Math.abs(result.getYaw()), ANGLE_EPSILON,
//                     "Yaw should be +-180-degrees when target is directly in -X direction");
//         }

//         /**
//          * A target at 45-degrees in the +X/+Y quadrant should have yaw near 45-degrees.
//          */
//         @Test
//         void yawIs45_WhenTargetIsDiagonal() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(5, 5, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertEquals(45.0, result.getYaw(), ANGLE_EPSILON,
//                     "Yaw should be ~45-degrees for a diagonal target in +X/+Y");
//         }

//         /**
//          * The computed velocity must be within the specified min/max speed bounds.
//          */
//         @Test
//         void velocityIsWithinBounds() throws Exception {
//             MutVector result = new MutVector();
//             double minSpeed = 1.0;
//             double maxSpeed = 50.0;
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(4, 0, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, minSpeed, maxSpeed);

//             double speed = result.getMagnitude();
//             assertTrue(speed >= minSpeed && speed <= maxSpeed,
//                     String.format("Speed %.2f must be within [%.2f, %.2f]", speed, minSpeed, maxSpeed));
//         }

//         /**
//          * The selected pitch angle should be clamped between minAngle and maxAngle.
//          */
//         @Test
//         void pitchIsWithinConfiguredBounds() throws Exception {
//             MutVector result = new MutVector();
//             double minAngle = 15.0;
//             double maxAngle = 45.0;
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(6, 0, 2.5, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, minAngle, maxAngle, 0, 100);

//             double pitch = result.getPitch();
//             assertTrue(pitch >= minAngle - ANGLE_EPSILON && pitch <= maxAngle + ANGLE_EPSILON,
//                     String.format("Pitch %.2f should be within [%.2f, %.2f]", pitch, minAngle, maxAngle));
//         }

//         /**
//          * Throws when the required speed exceeds maxSpeed.
//          */
//         @Test
//         void throwsWhenSpeedExceedsMax() {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(50, 0, 10.0, Rotation3d.kZero);

//             assertThrows(Exception.class,
//                     () -> TurretUtilities.calculateAimVector(result, source, target,
//                             SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 2.0),
//                     "Should throw when calculated speed exceeds maxSpeed");
//         }

//         /**
//          * For a target at nearly the same effective height, the optimal angle
//          * from horizontal is near 45-degrees, so the "from vertical" pitch should be near 45-degrees.
//          */
//         @Test
//         void sameHeightTarget_ProducesReasonableTrajectory() throws Exception {
//             MutVector result = new MutVector();
//             double targetZ = SHOOTER_HEIGHT - OVERSHOOT_HEIGHT;
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(5, 0, targetZ, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);

//             double pitch = result.getPitch();
//             assertTrue(pitch > 30 && pitch < 65,
//                     String.format("For same-height shot, pitch should be near 45-degrees from vertical, got %.2f", pitch));
//             assertTrue(result.getMagnitude() > 0, "Speed should be positive");
//         }

//         /**
//          * A higher target should result in a steeper launch angle (lower pitch "from vertical" value).
//          */
//         @Test
//         void higherTarget_ProducesSteeper_LaunchAngle() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d lowTarget = new Pose3d(5, 0, 1.5, Rotation3d.kZero);
//             Pose3d highTarget = new Pose3d(5, 0, 4.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, lowTarget,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);
//             double lowPitch = result.getPitch();

//             TurretUtilities.calculateAimVector(result, source, highTarget,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);
//             double highPitch = result.getPitch();

//             assertTrue(highPitch < lowPitch,
//                     String.format("Higher target pitch (%.2f) should be less than lower target pitch (%.2f)",
//                             highPitch, lowPitch));
//         }

//         /**
//          * A farther target (same height) should require more speed.
//          */
//         @Test
//         void fartherTarget_RequiresMoreSpeed() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d nearTarget = new Pose3d(3, 0, 2.0, Rotation3d.kZero);
//             Pose3d farTarget = new Pose3d(8, 0, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, nearTarget,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);
//             double nearSpeed = result.getMagnitude();

//             TurretUtilities.calculateAimVector(result, source, farTarget,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);
//             double farSpeed = result.getMagnitude();

//             assertTrue(farSpeed > nearSpeed,
//                     String.format("Far shot speed (%.2f) should exceed near shot speed (%.2f)",
//                             farSpeed, nearSpeed));
//         }

//         /**
//          * The calculated vector's yaw should match the bearing from source to target.
//          */
//         @Test
//         void vectorYawMatchesBearingToTarget() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(1, 2, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(6, 7, 2.5, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);

//             double expectedYaw = Math.toDegrees(Math.atan2(
//                     target.getY() - source.getY(),
//                     target.getX() - source.getX()));

//             assertEquals(expectedYaw, result.getYaw(), ANGLE_EPSILON,
//                     "Vector yaw should match bearing from source to target");
//         }

//         /**
//          * Verify projectile physics: the computed velocity and angle should deliver
//          * the projectile to the effective target height at the target's horizontal distance.
//          */
//         @Test
//         void projectileReachesTargetHeight_AtTargetDistance() throws Exception {
//             MutVector result = new MutVector();
//             double distance = 6.0;
//             double targetZ = 2.5;
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(distance, 0, targetZ, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 5, 80, 0, 200);

//             double speed = result.getMagnitude();
//             double pitchFromVertical = result.getPitch();
//             double physicsAngleRad = Math.toRadians(90.0 - pitchFromVertical);

//             double vHorizontal = speed * Math.cos(physicsAngleRad);
//             double vVertical = speed * Math.sin(physicsAngleRad);

//             double flightTime = distance / vHorizontal;

//             double effectiveTargetHeight = targetZ + OVERSHOOT_HEIGHT;
//             double finalHeight = SHOOTER_HEIGHT + vVertical * flightTime
//                     - 0.5 * PhysicsConstants.GRAVITY * flightTime * flightTime;

//             assertEquals(effectiveTargetHeight, finalHeight, 0.05,
//                     String.format("Projectile at d=%.1fm should reach h=%.2fm, got %.2fm",
//                             distance, effectiveTargetHeight, finalHeight));
//         }

//         /**
//          * Zero horizontal distance should throw (degenerate geometry).
//          */
//         @Test
//         void throwsForZeroDistance() {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(3, 3, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(3, 3, 5, Rotation3d.kZero);

//             assertThrows(Exception.class,
//                     () -> TurretUtilities.calculateAimVector(result, source, target,
//                             SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100),
//                     "Zero horizontal distance should produce a degenerate calculation");
//         }

//         /**
//          * Second quadrant target (-X, +Y) should produce yaw around 135-degrees.
//          */
//         @Test
//         void yawIsCorrect_WhenTargetIsInSecondQuadrant() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(-5, 5, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertEquals(135.0, result.getYaw(), ANGLE_EPSILON,
//                     "Yaw should be ~135-degrees for target in -X/+Y quadrant");
//         }

//         /**
//          * Fourth quadrant target (+X, -Y) should produce yaw around -45-degrees.
//          */
//         @Test
//         void yawIsCorrect_WhenTargetIsInFourthQuadrant() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(5, -5, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertEquals(-45.0, result.getYaw(), ANGLE_EPSILON,
//                     "Yaw should be ~-45-degrees for target in +X/-Y quadrant");
//         }

//         /**
//          * The result should be written to the provided MutVector, not an internal field.
//          */
//         @Test
//         void writesResultToProvidedVector() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);
//             Pose3d target = new Pose3d(5, 0, 2.0, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source, target,
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);

//             assertTrue(result.getMagnitude() > 0, "Result vector should have non-zero magnitude");
//             assertEquals(0.0, result.getYaw(), ANGLE_EPSILON, "Yaw should be written to result");
//         }

//         /**
//          * Successive calls with different targets should overwrite the result vector.
//          */
//         @Test
//         void overwritesResultOnSuccessiveCalls() throws Exception {
//             MutVector result = new MutVector();
//             Pose3d source = new Pose3d(0, 0, SHOOTER_HEIGHT, Rotation3d.kZero);

//             TurretUtilities.calculateAimVector(result, source,
//                     new Pose3d(5, 0, 2.0, Rotation3d.kZero),
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);
//             double yaw1 = result.getYaw();

//             TurretUtilities.calculateAimVector(result, source,
//                     new Pose3d(0, 5, 2.0, Rotation3d.kZero),
//                     SHOOTER_HEIGHT, OVERSHOOT_HEIGHT, 10, 60, 0, 100);
//             double yaw2 = result.getYaw();

//             assertEquals(0.0, yaw1, ANGLE_EPSILON);
//             assertEquals(90.0, yaw2, ANGLE_EPSILON);
//         }
//     }

//     // =========================================================================
//     // calculateSensorPose tests
//     // =========================================================================

//     @Nested
//     class CalculateSensorPoseTests {

//         private static final Translation3d TURRET_ORIGIN = new Translation3d(
//                 Units.inchesToMeters(8.25),
//                 Units.inchesToMeters(5.75),
//                 Units.inchesToMeters(15.894));

//         /**
//          * With a forward offset and zero turret rotation,
//          * the sensor X should be origin + offset.
//          */
//         @Test
//         void forwardOffset_AddsToX_AtZeroRotation() {
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0.1, 0.0, 0.0, Rotation3d.kZero),
//                     0);

//             assertEquals(TURRET_ORIGIN.getX() + 0.1, result.getX(), EPSILON);
//             assertEquals(TURRET_ORIGIN.getY(), result.getY(), EPSILON);
//         }

//         /**
//          * With a lateral offset and zero rotation, Y should increase.
//          */
//         @Test
//         void lateralOffset_AddsToY_AtZeroRotation() {
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0.0, 0.05, 0.0, Rotation3d.kZero),
//                     0);

//             assertEquals(TURRET_ORIGIN.getX(), result.getX(), EPSILON);
//             assertEquals(TURRET_ORIGIN.getY() + 0.05, result.getY(), EPSILON);
//         }

//         /**
//          * At 90-degrees turret rotation, a forward offset should map to +Y.
//          */
//         @Test
//         void forwardOffset_MapsToY_At90DegRotation() {
//             double rot = Math.toRadians(90);
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0.1, 0.0, 0.0, Rotation3d.kZero),
//                     rot);

//             assertEquals(TURRET_ORIGIN.getX(), result.getX(), EPSILON);
//             assertEquals(TURRET_ORIGIN.getY() + 0.1, result.getY(), EPSILON);
//         }

//         /**
//          * At 90-degrees turret rotation, a lateral (Y) offset should map to -X.
//          */
//         @Test
//         void lateralOffset_MapsToNegX_At90DegRotation() {
//             double rot = Math.toRadians(90);
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0.0, 0.05, 0.0, Rotation3d.kZero),
//                     rot);

//             assertEquals(TURRET_ORIGIN.getX() - 0.05, result.getX(), EPSILON);
//             assertEquals(TURRET_ORIGIN.getY(), result.getY(), EPSILON);
//         }

//         /**
//          * At 180-degrees rotation, a forward offset should negate in X.
//          */
//         @Test
//         void forwardOffset_NegatesInX_At180DegRotation() {
//             double rot = Math.toRadians(180);
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0.1, 0.0, 0.0, Rotation3d.kZero),
//                     rot);

//             assertEquals(TURRET_ORIGIN.getX() - 0.1, result.getX(), EPSILON);
//             assertEquals(TURRET_ORIGIN.getY(), result.getY(), EPSILON);
//         }

//         /**
//          * Combined forward + lateral offset at 45-degrees rotation.
//          */
//         @Test
//         void combinedOffset_RotatesCorrectly_At45Deg() {
//             double angle = Math.toRadians(45);
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0.1, 0.05, 0.0, Rotation3d.kZero),
//                     angle);

//             double expectedDx = 0.1 * Math.cos(angle) - 0.05 * Math.sin(angle);
//             double expectedDy = 0.1 * Math.sin(angle) + 0.05 * Math.cos(angle);

//             assertEquals(TURRET_ORIGIN.getX() + expectedDx, result.getX(), EPSILON);
//             assertEquals(TURRET_ORIGIN.getY() + expectedDy, result.getY(), EPSILON);
//         }

//         /**
//          * Z offset is independent of turret rotation.
//          */
//         @Test
//         void zOffset_IsIndependent_OfRotation() {
//             double expectedZ = TURRET_ORIGIN.getZ() + 0.08;
//             double[] angles = { 0, Math.PI / 4, Math.PI / 2, Math.PI, -Math.PI / 3 };

//             for (double angle : angles) {
//                 Pose3d result = TurretUtilities.calculateSensorPose(
//                         TURRET_ORIGIN,
//                         new Transform3d(0.1, 0.05, 0.08, Rotation3d.kZero),
//                         angle);
//                 assertEquals(expectedZ, result.getZ(), EPSILON,
//                         String.format("Z should be constant at angle=%.2f rad", angle));
//             }
//         }

//         /**
//          * XY offset magnitude should be preserved across rotations.
//          */
//         @Test
//         void offsetMagnitude_IsPreservedAcrossRotations() {
//             double expectedMagnitude = Math.hypot(0.15, 0.08);
//             double[] angles = { 0, Math.PI / 6, Math.PI / 3, Math.PI / 2, Math.PI, -Math.PI / 4, 2 * Math.PI };

//             for (double angle : angles) {
//                 Pose3d result = TurretUtilities.calculateSensorPose(
//                         TURRET_ORIGIN,
//                         new Transform3d(0.15, 0.08, 0.0, Rotation3d.kZero),
//                         angle);

//                 double dx = result.getX() - TURRET_ORIGIN.getX();
//                 double dy = result.getY() - TURRET_ORIGIN.getY();
//                 assertEquals(expectedMagnitude, Math.hypot(dx, dy), EPSILON,
//                         String.format("XY offset magnitude should be preserved at angle=%.2f rad", angle));
//             }
//         }

//         /**
//          * Sensor yaw should be fixed yaw + turret rotation.
//          */
//         @Test
//         void sensorYaw_CombinesFixedAndTurretRotation() {
//             double fixedYaw = Math.toRadians(15);
//             double turretAngle = Math.toRadians(60);

//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     TURRET_ORIGIN,
//                     new Transform3d(0, 0, 0, new Rotation3d(0, 0, fixedYaw)),
//                     turretAngle);

//             assertEquals(fixedYaw + turretAngle, result.getRotation().getZ(), EPSILON);
//         }

//         /**
//          * Sensor pitch should be constant regardless of turret rotation.
//          */
//         @Test
//         void sensorPitch_IsConstant_AcrossRotations() {
//             double fixedPitch = Math.toRadians(25);
//             double[] angles = { 0, Math.PI / 4, Math.PI, -Math.PI / 2 };

//             for (double angle : angles) {
//                 Pose3d result = TurretUtilities.calculateSensorPose(
//                         TURRET_ORIGIN,
//                         new Transform3d(0, 0, 0, new Rotation3d(0, fixedPitch, 0)),
//                         angle);

//                 assertEquals(fixedPitch, result.getRotation().getY(), EPSILON,
//                         String.format("Pitch should be constant at turret angle=%.2f rad", angle));
//             }
//         }

//         /**
//          * With a different turret origin, the math should still work.
//          */
//         @Test
//         void worksWithArbitraryTurretOrigin() {
//             Translation3d customOrigin = new Translation3d(1.0, 2.0, 0.5);
//             Pose3d result = TurretUtilities.calculateSensorPose(
//                     customOrigin,
//                     new Transform3d(0.2, 0.0, 0.0, Rotation3d.kZero),
//                     0);

//             assertEquals(1.2, result.getX(), EPSILON);
//             assertEquals(2.0, result.getY(), EPSILON);
//             assertEquals(0.5, result.getZ(), EPSILON);
//         }
//     }
// }
