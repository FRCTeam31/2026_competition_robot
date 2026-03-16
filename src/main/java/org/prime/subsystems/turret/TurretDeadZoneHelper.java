package org.prime.subsystems.turret;

import org.littletonrobotics.junction.Logger;

/**
 * Utility for handling a turret dead zone -- a physical arc that the turret
 * cannot traverse (e.g., where wiring passes through).
 *
 * <p>All public methods work in <b>rotations</b> (1 rotation = 360-degrees) to match
 * CTRE TalonFX position units.  Internally the dead zone is defined by a
 * start and end angle (in degrees, 0-360); the zone spans clockwise from
 * start to end through the "forbidden" region.</p>
 *
 * <h3>Coordinate convention</h3>
 * <ul>
 *   <li>The <em>live zone</em> is the arc from {@code DEADZONE_END} clockwise
 *       through 360/0 to {@code DEADZONE_START} -- the region the turret may
 *       occupy.</li>
 *   <li>The <em>dead zone</em> is the complementary arc from {@code DEADZONE_START}
 *       to {@code DEADZONE_END}.</li>
 * </ul>
 *
 * <h3>Key behaviours</h3>
 * <ol>
 *   <li><b>Target remapping</b> -- if the desired setpoint (normalised to one
 *       rotation) falls inside the dead zone it is clamped to the nearer edge.</li>
 *   <li><b>Path legality</b> -- even when both current and target positions are
 *       in the live zone, the shortest-arc path may cross the dead zone.  In
 *       that case the helper returns a setpoint on the <em>same side</em> of
 *       the dead zone as the current position so the turret first travels to
 *       the edge, and the next cycle will continue around the legal way.</li>
 *   <li><b>Manual clamping</b> -- manual duty-cycle input is zeroed if it would
 *       drive the turret into the dead zone from an edge.</li>
 * </ol>
 */
@SuppressWarnings("unused")
public class TurretDeadZoneHelper {

    private final double _startRot; // dead zone start in rotations [0, 1)
    private final double _endRot; // dead zone end   in rotations [0, 1)
    private final double _dzSize; // dead zone span  in rotations (0, 1)
    private static final double EDGE_BUFFER = 0.005;

    /**
     * @param deadZoneStartDegrees Start of the dead zone in degrees [0, 360)
     * @param deadZoneEndDegrees   End of the dead zone in degrees [0, 360)
     */
    public TurretDeadZoneHelper(double deadZoneStartDegrees, double deadZoneEndDegrees) {
        _startRot = normalizeTo01(deadZoneStartDegrees / 360.0);
        _endRot = normalizeTo01(deadZoneEndDegrees / 360.0);
        _dzSize = normalizeTo01(_endRot - _startRot);
    }

    // ------------------------ public API ------------------------

    /**
     * Returns {@code true} when the normalised angle (mod 1 rotation) falls
     * inside the dead zone arc from start -> end.
     */
    public boolean isInDeadZone(double positionRotations) {
        double norm = normalizeTo01(positionRotations);
        return offsetFromStart(norm) < _dzSize;
    }

    /**
     * Given the turret's current (unwrapped) position and a desired target
     * position (also unwrapped, in rotations), returns the legal setpoint the
     * motor should actually target.
     *
     * <p>Rules applied in order:</p>
     * <ol>
     *   <li>If the target's normalised angle is inside the dead zone, clamp it
     *       to whichever edge is closer.</li>
     *   <li>If the shortest-arc path from current to target would cross the
     *       dead zone, re-route to the near edge of the dead zone (the turret
     *       will reach the edge this cycle, and next cycle the target will be
     *       legal from the new position).</li>
     *   <li>Otherwise return the target unchanged (adjusted to be within +-0.5
     *       rotations of the current position so MotionMagic takes the short way).</li>
     * </ol>
     *
     * @param currentRotations Current turret position (unwrapped rotations)
     * @param targetRotations  Desired turret position (unwrapped rotations)
     * @return The legal setpoint in unwrapped rotations
     */
    public double computeLegalSetpoint(double currentRotations, double targetRotations) {
        double currentNorm = normalizeTo01(currentRotations);
        double targetNorm = normalizeTo01(targetRotations);

        Logger.recordOutput("DeadZoneHelper/computeLegalSetpoint/targetInDeadZone", isInDeadZone(targetRotations));

        // If target is in the dead zone, clamp to the nearer edge
        if (isInDeadZone(targetRotations)) {
            targetNorm = closerEdge(targetNorm);
        }

        // Compute shortest-arc delta from current to target (-0.5 .. +0.5)
        double delta = shortestDelta(currentNorm, targetNorm);

        // Check whether this arc crosses the dead zone
        var arcCrossesDeadZone = arcCrossesDeadZone(currentNorm, delta);
        Logger.recordOutput("DeadZoneHelper/computeLegalSetpoint/arcCrossesDeadZone", arcCrossesDeadZone);
        if (arcCrossesDeadZone) {
            // Short arc is blocked - reverse direction to take the legal long arc
            double longDelta = delta > 0 ? delta - 1.0 : delta + 1.0;
            return normalizeTo01(currentRotations + longDelta);
        }

        // Convert back to unwrapped rotations: keep same "integer turns" as current,
        // then add the delta
        return normalizeTo01(currentRotations + delta);
    }

    /**
     * Determines whether the given manual yaw input should be blocked to
     * prevent the turret from rotating into the dead zone.
     *
     * <p>The turret is always allowed to rotate <em>out</em> of the dead zone.
     * Only the direction that would drive deeper into (or into) the dead zone
     * is blocked.</p>
     *
     * <ul>
     *   <li>If the turret is at or past the <b>start</b> edge (positive side),
     *       positive input (which would push further into the DZ) is blocked.</li>
     *   <li>If the turret is at or past the <b>end</b> edge (negative side),
     *       negative input (which would push further into the DZ) is blocked.</li>
     * </ul>
     *
     * @param currentRotations Current turret position in rotations
     * @param manualInput      Raw operator input (-1 to +1, positive = increasing rotation)
     * @return {@code true} if the input should be blocked (zeroed), {@code false} if it is safe
     */
    public boolean shouldBlockManualInput(double currentRotations, double manualInput) {
        if (manualInput == 0)
            return false;

        double norm = normalizeTo01(currentRotations);
        double offset = offsetFromStart(norm); // 0..1, values < _dzSize are in DZ

        // REPLACE with this:
        if (!isInDeadZone(currentRotations)) {
            double distToStart = normalizeTo01(_startRot - norm);
            double distToEnd = normalizeTo01(norm - _endRot);

            // Block positive input when approaching the start edge
            if (distToStart < EDGE_BUFFER && manualInput > 0)
                return true;
            // Block negative input when approaching the end edge
            if (distToEnd < EDGE_BUFFER && manualInput < 0)
                return true;

            return false;
        }

        // We're inside the dead zone. Determine which half we're in to decide
        // which direction is "out".
        // Closer to start edge -> negative input goes out, positive goes deeper in
        // Closer to end edge   -> positive input goes out, negative goes deeper in
        boolean closerToStart = offset < _dzSize / 2.0;

        if (closerToStart && manualInput > 0) {
            // Would drive deeper into dead zone from the start side
            return true;
        }
        if (!closerToStart && manualInput < 0) {
            // Would drive deeper into dead zone from the end side
            return true;
        }

        // The input is driving OUT of the dead zone -- allow it
        return false;
    }

    /**
     * Returns the dead zone start angle in rotations [0, 1).
     */
    public double getStartRotations() {
        return _startRot;
    }

    /**
     * Returns the dead zone end angle in rotations [0, 1).
     */
    public double getEndRotations() {
        return _endRot;
    }

    // ------------------------ internals -------------------------

    /**
     * Normalises any rotation value into [0, 1).
     */
    static double normalizeTo01(double rotations) {
        double mod = rotations % 1.0;
        return mod < 0 ? mod + 1.0 : mod;
    }

    /**
     * Shortest signed delta from {@code from} to {@code to}, both in [0,1).
     * Result is in (-0.5, +0.5].
     */
    private double shortestDelta(double from, double to) {
        double d = normalizeTo01(to - from);
        return d > 0.5 ? d - 1.0 : d;
    }

    /**
     * How far {@code norm} is past {@code _startRot} in the positive direction
     * (both in [0,1)), returned in [0, 1). Values < _dzSize are inside the dead zone.
     */
    private double offsetFromStart(double norm) {
        return normalizeTo01(norm - _startRot);
    }

    /**
     * Returns whichever dead zone edge (start or end) is angularly closer to
     * the given normalised position.
     */
    private double closerEdge(double norm) {
        double offset = offsetFromStart(norm); // 0..dzSize (inside DZ)
        return offset < _dzSize / 2.0 ? _startRot : _endRot;
    }

    /**
     * Returns the dead zone edge that the turret can reach from {@code currentNorm}
     * <em>without</em> crossing the dead zone.  This is the edge on the "current
     * side" of the dead zone.
     *
     * <p>If current is closer to start going backwards, return start.
     * If current is closer to end going forwards, return end.</p>
     */
    private double nearEdgeFor(double currentNorm) {
        // Distance going positive (CW) to dead zone start
        double distToStart = normalizeTo01(_startRot - currentNorm);
        // Distance going negative (CCW) to dead zone end
        double distToEnd = normalizeTo01(currentNorm - _endRot);
        return distToStart <= distToEnd ? _startRot : _endRot;
    }

    /**
     * Returns {@code true} when the arc from {@code start} (normalised) in the
     * direction of {@code delta} (signed, magnitude <= 0.5) crosses any part of
     * the dead zone.
     */
    private boolean arcCrossesDeadZone(double startNorm, double delta) {
        if (Math.abs(delta) < 1e-9)
            return false;

        // Check if the endpoint lands inside the dead zone
        double endNorm = normalizeTo01(startNorm + delta);
        if (offsetFromStart(endNorm) < _dzSize)
            return true;

        if (delta > 0) {
            // CW sweep: does either dead zone edge fall strictly inside the arc?
            double toStart = normalizeTo01(_startRot - startNorm);
            double toEnd = normalizeTo01(_endRot - startNorm);
            return (toStart > 1e-9 && toStart < delta)
                    || (toEnd > 1e-9 && toEnd < delta);
        } else {
            // CCW sweep: same, measured backward from startNorm
            double absDelta = -delta;
            double toEnd = normalizeTo01(startNorm - _endRot);
            double toStart = normalizeTo01(startNorm - _startRot);
            return (toEnd > 1e-9 && toEnd < absDelta)
                    || (toStart > 1e-9 && toStart < absDelta);
        }
    }
}
