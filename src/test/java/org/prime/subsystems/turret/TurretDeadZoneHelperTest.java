// package org.prime.subsystems.turret;

// import static org.junit.jupiter.api.Assertions.*;

// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Nested;
// import org.junit.jupiter.api.Test;

// import edu.wpi.first.hal.HAL;

// /**
//  * Tests for {@link TurretDeadZoneHelper} dead zone math.
//  * All angles are in rotations (1 rotation = 360-degrees) unless otherwise noted.
//  */
// class TurretDeadZoneHelperTest {

//     private static final double EPSILON = 1e-6;

//     @BeforeEach
//     void setUp() {
//         HAL.initialize(500, 0);
//     }

//     // -------------------- Default helper: 170-degrees-190-degrees dead zone --------------------

//     /** A 20-degrees dead zone centred on 180-degrees (the "back" of the turret). */
//     private TurretDeadZoneHelper makeDefault() {
//         return new TurretDeadZoneHelper(170, 190);
//     }

//     // --------------------------- isInDeadZone ------------------------------------

//     @Nested
//     class IsInDeadZone {

//         @Test
//         void centerOfDeadZone_isInside() {
//             var helper = makeDefault();
//             assertTrue(helper.isInDeadZone(180.0 / 360.0)); // 180-degrees
//         }

//         @Test
//         void startEdge_isInside() {
//             var helper = makeDefault();
//             // The start edge itself is the first point of the dead zone
//             assertTrue(helper.isInDeadZone(170.0 / 360.0));
//         }

//         @Test
//         void justBeforeStart_isOutside() {
//             var helper = makeDefault();
//             assertFalse(helper.isInDeadZone(169.0 / 360.0));
//         }

//         @Test
//         void justAfterEnd_isOutside() {
//             var helper = makeDefault();
//             assertFalse(helper.isInDeadZone(191.0 / 360.0));
//         }

//         @Test
//         void zeroDegreesIsOutside() {
//             var helper = makeDefault();
//             assertFalse(helper.isInDeadZone(0));
//         }

//         @Test
//         void negativeAngle_normalisedCorrectly() {
//             var helper = makeDefault();
//             // -180-degrees is the same as 180-degrees = inside
//             assertTrue(helper.isInDeadZone(-180.0 / 360.0));
//         }

//         @Test
//         void multipleRotations_normalisedCorrectly() {
//             var helper = makeDefault();
//             // 540-degrees = 180-degrees = inside
//             assertTrue(helper.isInDeadZone(540.0 / 360.0));
//         }
//     }

//     // ------------------ Wrapping dead zone (e.g., 350deg-10deg) ----------------------

//     @Nested
//     class WrappingDeadZone {

//         private TurretDeadZoneHelper makeWrapping() {
//             return new TurretDeadZoneHelper(350, 10);
//         }

//         @Test
//         void zeroIsInside() {
//             assertTrue(makeWrapping().isInDeadZone(0));
//         }

//         @Test
//         void fiveDegreesIsInside() {
//             assertTrue(makeWrapping().isInDeadZone(5.0 / 360.0));
//         }

//         @Test
//         void fifteenDegreesIsOutside() {
//             assertFalse(makeWrapping().isInDeadZone(15.0 / 360.0));
//         }

//         @Test
//         void threeFortyFiveIsOutside() {
//             assertFalse(makeWrapping().isInDeadZone(345.0 / 360.0));
//         }

//         @Test
//         void threeHundredFiftyFiveIsInside() {
//             assertTrue(makeWrapping().isInDeadZone(355.0 / 360.0));
//         }
//     }

//     // ---------------------- computeLegalSetpoint ---------------------------------

//     @Nested
//     class ComputeLegalSetpoint {

//         @Test
//         void targetInLiveZone_noDeadZoneCrossing_returnsTarget() {
//             var helper = makeDefault();
//             // Current at 90-degrees, target at 100-degrees -- simple move, no crossing
//             double current = 90.0 / 360.0;
//             double target = 100.0 / 360.0;
//             double result = helper.computeLegalSetpoint(current, target);
//             assertEquals(target, result, EPSILON);
//         }

//         @Test
//         void targetInsideDeadZone_clampedToNearerEdge() {
//             var helper = makeDefault();
//             // Current at 160-degrees, target at 175-degrees (inside DZ) -> should clamp to 170-degrees (start edge)
//             double current = 160.0 / 360.0;
//             double target = 175.0 / 360.0;
//             double result = helper.computeLegalSetpoint(current, target);
//             assertEquals(170.0 / 360.0, result, 0.01);
//         }

//         @Test
//         void targetInsideDeadZone_clampedToFarEdge() {
//             var helper = makeDefault();
//             // Current at 200-degrees, target at 185-degrees (inside DZ) -> should clamp to 190-degrees (end edge)
//             double current = 200.0 / 360.0;
//             double target = 185.0 / 360.0;
//             double result = helper.computeLegalSetpoint(current, target);
//             assertEquals(190.0 / 360.0, result, 0.01);
//         }

//         @Test
//         void shortestPathCrossesDeadZone_goesLongWay() {
//             var helper = makeDefault();
//             // Current at 160-degrees, target at 200-degrees. Shortest arc is 40-degrees through the DZ.
//             // Should NOT go through the DZ; should route to the DZ edge on current side.
//             double current = 160.0 / 360.0;
//             double target = 200.0 / 360.0;
//             double result = helper.computeLegalSetpoint(current, target);
//             // Should route to start edge (170-degrees) which is on the same side as current (160-degrees)
//             assertEquals(170.0 / 360.0, result, 0.01);
//         }

//         @Test
//         void longPathDoesNotCrossDeadZone_takesShortPath() {
//             var helper = makeDefault();
//             // Current at 0-degrees, target at 90-degrees -- 90-degrees arc doesn't cross 170-190 DZ
//             double current = 0.0;
//             double target = 90.0 / 360.0;
//             double result = helper.computeLegalSetpoint(current, target);
//             assertEquals(target, result, EPSILON);
//         }

//         @Test
//         void unwrappedCoordinatesPreserved() {
//             var helper = makeDefault();
//             // Current at 1.25 rotations (450-degrees = 90-degrees), target at 1.0 rotation (360-degrees = 0-degrees)
//             // The arc doesn't cross the DZ, so it should return something near current
//             double current = 1.25;
//             double target = 1.0;
//             double result = helper.computeLegalSetpoint(current, target);
//             // Result should be near current in unwrapped space (current + delta)
//             assertEquals(1.0, result, 0.01);
//         }
//     }

//     // ---------------------- shouldBlockManualInput -----------------------------

//     @Nested
//     class ShouldBlockManualInput {

//         @Test
//         void outsideDeadZone_neverBlocks() {
//             var helper = makeDefault();
//             // At 90-degrees (well outside DZ), any input is fine
//             assertFalse(helper.shouldBlockManualInput(90.0 / 360.0, 0.8));
//             assertFalse(helper.shouldBlockManualInput(90.0 / 360.0, -0.8));
//         }

//         @Test
//         void inDeadZone_nearStartEdge_positiveInputBlocked() {
//             var helper = makeDefault();
//             // At 172-degrees (inside DZ, closer to 170-degrees start edge), positive = deeper in -> blocked
//             assertTrue(helper.shouldBlockManualInput(172.0 / 360.0, 0.5));
//         }

//         @Test
//         void inDeadZone_nearStartEdge_negativeInputAllowed() {
//             var helper = makeDefault();
//             // At 172-degrees (inside DZ, closer to 170-degrees start edge), negative = back out -> allowed
//             assertFalse(helper.shouldBlockManualInput(172.0 / 360.0, -0.5));
//         }

//         @Test
//         void inDeadZone_nearEndEdge_negativeInputBlocked() {
//             var helper = makeDefault();
//             // At 188-degrees (inside DZ, closer to 190-degrees end edge), negative = deeper in -> blocked
//             assertTrue(helper.shouldBlockManualInput(188.0 / 360.0, -0.5));
//         }

//         @Test
//         void inDeadZone_nearEndEdge_positiveInputAllowed() {
//             var helper = makeDefault();
//             // At 188-degrees (inside DZ, closer to 190-degrees end edge), positive = out toward end -> allowed
//             assertFalse(helper.shouldBlockManualInput(188.0 / 360.0, 0.5));
//         }

//         @Test
//         void zeroInput_neverBlocks() {
//             var helper = makeDefault();
//             // Even inside the dead zone, zero input is not blocked
//             assertFalse(helper.shouldBlockManualInput(180.0 / 360.0, 0.0));
//         }

//         @Test
//         void exactlyAtStartEdge_positiveInputBlocked() {
//             var helper = makeDefault();
//             // At 170-degrees (start edge, inside DZ, closer to start), positive -> deeper in
//             assertTrue(helper.shouldBlockManualInput(170.0 / 360.0, 1.0));
//         }

//         @Test
//         void exactlyAtStartEdge_negativeInputAllowed() {
//             var helper = makeDefault();
//             // At 170-degrees (start edge), negative -> out of DZ
//             assertFalse(helper.shouldBlockManualInput(170.0 / 360.0, -1.0));
//         }
//     }

//     // ------------------------ normalizeTo01 --------------------------------------

//     @Nested
//     class NormalizeTo01 {

//         @Test
//         void positiveValue() {
//             assertEquals(0.25, TurretDeadZoneHelper.normalizeTo01(0.25), EPSILON);
//         }

//         @Test
//         void negativeValue() {
//             assertEquals(0.75, TurretDeadZoneHelper.normalizeTo01(-0.25), EPSILON);
//         }

//         @Test
//         void greaterThanOne() {
//             assertEquals(0.5, TurretDeadZoneHelper.normalizeTo01(1.5), EPSILON);
//         }

//         @Test
//         void exactlyZero() {
//             assertEquals(0.0, TurretDeadZoneHelper.normalizeTo01(0.0), EPSILON);
//         }

//         @Test
//         void multipleRotations() {
//             assertEquals(0.1, TurretDeadZoneHelper.normalizeTo01(3.1), EPSILON);
//         }
//     }
// }
