# FRC Shooting System -- Two-Phase Implementation Plan

## Context

The robot has a flywheel shooter with two degrees of freedom:
- **Hood angle** -- adjustable between `HOOD_MIN_ANGLE_DEGREES` and `HOOD_MAX_ANGLE_DEGREES`
- **Flywheel speed** -- adjustable between a min and max RPS

The current ballistics solver (`_mutNominalTargetVector`) produces a physically correct launch vector, but the raw magnitude is not usable as a flywheel setpoint due to energy losses and ball compression. Hood pitch from the vector is also inverted relative to what the mechanism needs. Both axes need to be driven from measured treemaps instead.

Yaw logic is correct and should not be changed.

---

## Phase 1 -- Max Hood Angle, Single Treemap

### Goal
Get the robot scoring as fast as possible using a single measured treemap at max hood angle.

### Measurement Protocol

On the practice field, fix the hood at `HOOD_MAX_ANGLE_DEGREES` for all measurements.

Take **5 distance samples** spanning your realistic shot range. Suggested distances (adjust to field geometry):

| Sample | Distance (m) |
|--------|-------------|
| 1      | 1.0         |
| 2      | 2.0         |
| 3      | 3.0         |
| 4      | 4.5         |
| 5      | 6.0         |

For each sample:
1. Place robot at known distance (tape measure or painted field markings)
2. Confirm hood is at max angle
3. Tune flywheel RPS live until shots are consistently going in
4. Record the confirmed RPS value

This takes ~10-15 minutes on the practice field.

### Code Changes

#### `TurretMap.java` -- add treemap

```java
public static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_MAX_HOOD = new InterpolatingDoubleTreeMap();
static {
    // TODO: fill in from Phase 1 measurements
    FLYWHEEL_SPEED_MAX_HOOD.put(1.0, 0.0);
    FLYWHEEL_SPEED_MAX_HOOD.put(2.0, 0.0);
    FLYWHEEL_SPEED_MAX_HOOD.put(3.0, 0.0);
    FLYWHEEL_SPEED_MAX_HOOD.put(4.5, 0.0);
    FLYWHEEL_SPEED_MAX_HOOD.put(6.0, 0.0);
}

public static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_MIN_HOOD = new InterpolatingDoubleTreeMap();
// Leave empty until Phase 2

public static double maxHoodMapMinDistance() { return FLYWHEEL_SPEED_MAX_HOOD.firstKey(); }
public static double maxHoodMapMaxDistance() { return FLYWHEEL_SPEED_MAX_HOOD.lastKey(); }
public static double minHoodMapMinDistance() { return FLYWHEEL_SPEED_MIN_HOOD.firstKey(); }
public static double minHoodMapMaxDistance() { return FLYWHEEL_SPEED_MIN_HOOD.lastKey(); }
```

#### New `ShotSolution.java` record

```java
public enum ShotState { CALCULATED, NOT_CALCULATED }

public record ShotSolution(
    double flywheelRPS,
    double hoodAngleDegrees,
    ShotState state
) {
    public static ShotSolution notCalculated() {
        return new ShotSolution(0, TurretMap.HOOD_MAX_ANGLE_DEGREES, ShotState.NOT_CALCULATED);
    }
}
```

#### New `resolveShotSolution()` method

Add to the turret subsystem. This is the only method that needs to change between phases.

```java
/**
 * Resolves flywheel speed and hood angle for a given distance.
 * Prefers max hood angle. Falls back to min hood for longer distances.
 * Returns NOT_CALCULATED if out of all known ranges.
 */
private ShotSolution resolveShotSolution(double distanceMeters) {
    if (!TurretMap.FLYWHEEL_SPEED_MAX_HOOD.isEmpty()
            && distanceMeters >= TurretMap.maxHoodMapMinDistance()
            && distanceMeters <= TurretMap.maxHoodMapMaxDistance()) {
        return new ShotSolution(
            TurretMap.FLYWHEEL_SPEED_MAX_HOOD.get(distanceMeters),
            TurretMap.HOOD_MAX_ANGLE_DEGREES,
            ShotState.CALCULATED
        );
    }

    if (!TurretMap.FLYWHEEL_SPEED_MIN_HOOD.isEmpty()
            && distanceMeters >= TurretMap.minHoodMapMinDistance()
            && distanceMeters <= TurretMap.minHoodMapMaxDistance()) {
        return new ShotSolution(
            TurretMap.FLYWHEEL_SPEED_MIN_HOOD.get(distanceMeters),
            TurretMap.HOOD_MIN_ANGLE_DEGREES,
            ShotState.CALCULATED
        );
    }

    return ShotSolution.notCalculated();
}
```

#### `actOnAutoMode()` -- replace hood and flywheel blocks

```java
// Distance from turret pose to target
double distance = target.pose.getTranslation().toTranslation2d()
    .getDistance(turretPose.getTranslation().toTranslation2d());

var shot = resolveShotSolution(distance);

// Hood
_turret.controlHood(Degrees.of(shot.hoodAngleDegrees()));

// Flywheel
if (inputs.FiringState == FiringState.FIRING) {
    if (shot.state() == ShotState.NOT_CALCULATED) {
        _turret.setFeederSpeed(0); // never shoot blind
        recordOutput("ShotState", "NOT_CALCULATED");
        return;
    }
    recordOutput("ShotState", "CALCULATED");
    _turret.controlFlywheel(RadiansPerSecond.of(shot.flywheelRPS() * 2 * Math.PI));
}
```

> **Note:** Verify whether `controlFlywheel` expects rad/s or RPS and adjust accordingly.

---

## Phase 2 -- Min Hood Angle, Extended Range

### Goal
Add long-distance shots by taking a second set of measurements at `HOOD_MIN_ANGLE_DEGREES`.

### When to do this
After Phase 1 is confirmed working in a match. Use the next available practice field slot.

### Measurement Protocol

Same procedure as Phase 1, but:
- Fix hood at `HOOD_MIN_ANGLE_DEGREES` for all measurements
- Focus on distances **beyond the max of your Phase 1 map** (e.g. 6m-10m)
- Suggested samples: 6.0m, 7.5m, 9.0m (adjust to your field geometry)

> The bottom of the min-hood range can overlap slightly with the top of the max-hood range. The resolution logic will always prefer max hood when both maps cover a distance.

### Code Changes

Populate `FLYWHEEL_SPEED_MIN_HOOD` in `TurretMap.java`:

```java
static {
    // TODO: fill in from Phase 2 measurements
    FLYWHEEL_SPEED_MIN_HOOD.put(6.0, 0.0);
    FLYWHEEL_SPEED_MIN_HOOD.put(7.5, 0.0);
    FLYWHEEL_SPEED_MIN_HOOD.put(9.0, 0.0);
}
```

No other changes needed. `resolveShotSolution()` will automatically start using the min hood map once it is populated, because the `isEmpty()` guard lifts.

---

## Shot Resolution Logic Summary

```
Given distanceMeters:
  ├─ Is distance within FLYWHEEL_SPEED_MAX_HOOD range?
  │    └─ YES -> return (maxHoodRPS, HOOD_MAX_ANGLE) [CALCULATED]
  ├─ Is FLYWHEEL_SPEED_MIN_HOOD populated AND distance within its range?
  │    └─ YES -> return (minHoodRPS, HOOD_MIN_ANGLE) [CALCULATED]
  └─ Otherwise -> return NOT_CALCULATED, feeder cut immediately
```

---

## Future Work (Post-Competition)

Once both treemaps are measured and validated, a blended two-DOF solution can be implemented inside `resolveShotSolution()` without touching any other code. The blending parameter `t` can be derived from:
- Minimizing flywheel delta from current speed (fastest shot readiness)
- Minimizing hood movement from current angle
- A weighted combination of both

This is purely an optimization of shot readiness time and does not affect shot accuracy, which is already baked into the treemap values.
