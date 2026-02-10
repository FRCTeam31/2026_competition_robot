# Copilot Instructions - FRC Team 31 (2026 Competition Robot)

## Project Overview
WPILib-based FRC robot code using **AdvantageKit** for logging/replay, **PathPlanner** for autonomous paths, **CTRE Phoenix 6** motors, and **REV** hardware. Java 17, Gradle build.

## Naming Conventions
- **Private fields**: `_underscoredCamelCase` (e.g., `_turret`, `_gyroInputs`)
- **Public fields**: `PascalCase` (e.g., `Kinematics`, `AutoChooser`)
- **Constants**: `SCREAMING_SNAKE_CASE` (e.g., `FLYWHEEL_MAX_SPEED`, `TURRET_GEAR_RATIO`)

## Architecture Patterns

### IO Abstraction Pattern (Critical)
Every subsystem uses a Real/Sim split with an interface. This enables simulation and replay:
```
subsystems/<name>/
├── I<Name>.java           # Interface defining hardware operations
├── <Name>.java            # SubsystemBase (commands, state machines)
├── <Name>Real.java        # Real hardware implementation
├── <Name>Sim.java         # Physics simulation implementation
├── <Name>Inputs.java      # @AutoLog annotated inputs class
└── <Name>Map.java         # Constants (CAN IDs, PID, gear ratios)
```

**Example pattern** from `ITurret.java`:
```java
public interface ITurret {
    public void updateInputs(TurretInputsAutoLogged inputs);
    public void controlFlywheel(ControlRequest controlRequest);
    // ...
}
```

Subsystems instantiate Real vs Sim based on `isReal` parameter from `Container.initialize(isReal())`.

### SuperStructure Pattern
`SuperStructure.java` holds **global state** as static `*InputsAutoLogged` instances. All subsystems read/write here:
```java
SuperStructure.Swerve.EstimatedRobotPose  // Pose2d from pose estimator
SuperStructure.Turret.FlywheelState       // Enum state machine
SuperStructure.Climb.climbControlState    // Climb sequence state
```

### Container Pattern
`Container.java` is the **dependency injection root**:
- Creates all subsystems, binds operator controls
- Registers PathPlanner named commands
- Contains compound commands combining multiple subsystems (e.g., `startShooting()`, `setupClimb()`)

### Subsystem Command Pattern (SOLID)
Subsystems expose **command factories** for state changes, never direct mechanism control:
```java
// ✓ Correct - expose commands
public Command setFlywheel(FlywheelState state) { ... }

// ✗ Wrong - don't expose direct motor control
public void setFlywheelVoltage(double volts) { ... }  // Keep private
```

## Key Conventions

### Inputs Classes (`@AutoLog`)
Always annotate with `@AutoLog` - AdvantageKit generates `*AutoLogged` subclasses:
```java
@AutoLog
public class TurretInputs {
    public Rotation2d TurretRotation = Rotation2d.kZero;
    public Turret.FlywheelState FlywheelState = Turret.FlywheelState.STOPPED;
}
```

### Map Classes (Constants)
All hardware constants go in `*Map.java` files:
- CAN IDs, gear ratios, PID constants
- Use `ExtendedPIDConstants` from `org.prime.control` (includes feedforward: kV, kA, kS)

### State Machines via Enums
Subsystems define state enums and expose command factories:
```java
public enum FlywheelState { IDLE, SHOOTING, STOPPED }
public Command setFlywheel(FlywheelState state) { ... }
```

### PathPlanner Named Commands
Subsystems that need autonomous integration expose a `getNamedCommands()` method returning `Map<String, Command>`. Register in `Container.java`:
```java
// In Swerve.java (or any subsystem)
public Map<String, Command> getNamedCommands() {
    return Map.of(
        "stopDrive", stopCommand(),
        "alignToTarget", autoAlignCommand()
    );
}

// In Container.java during initialization
NamedCommands.registerCommands(Swerve.getNamedCommands());
NamedCommands.registerCommands(Turret.getNamedCommands());
```
Command names must match exactly what's used in PathPlanner autonomous routines.

### Utility Classes (`org.prime.*`)
Reusable FRC utilities in `src/main/java/org/prime/`:
- `MutVector` - Mutable 3D vector for ballistics calculations (turret aiming)
- `ExtendedPIDConstants` - PID + feedforward constants
- `SwerveControlSuppliers` - Joystick input processing

## Turret Ballistics Math (`MutVector.setToTargetVector`)
The turret uses projectile motion physics to calculate shot vectors. Key method in `MutVector.java`:

```java
setToTargetVector(sourcePose, targetPose, minAngle, maxAngle, minSpeed, maxSpeed)
```

**Algorithm**: Iterates from `maxAngle` down to `minAngle` (0.5° steps), solving for launch velocity using:
```
v² = g / (2 * (d*sin(θ)*cos(θ) - Δh*cos²(θ)))
```
Where: `d` = horizontal distance, `Δh` = height difference, `θ` = launch angle, `g` = gravity

Returns the first valid solution within speed bounds. The `Turret.calculateTurretVectorFromRobotPose()` method then optionally applies motion compensation by subtracting robot velocity and turret tangent velocity vectors.

## Build & Test Commands
```bash
./gradlew build          # Compile and run tests
./gradlew test           # Run JUnit tests only
./gradlew deploy         # Deploy to roboRIO (requires team number)
./gradlew simulateJava   # Run robot simulation
```

Tests use HAL simulation - always call `HAL.initialize(500, 0)` in `@BeforeEach`.

## Swerve Drive Structure
`SwerveIOPackager` manages 4 modules + gyro, creates `SwerveDriveKinematics` and `SwerveDrivePoseEstimator`:
- Module order: FL, FR, RL, RR
- Gyro: Pigeon 2 (real) or `GyroSim`

## External Dependencies
| Dependency | Purpose |
|------------|---------|
| AdvantageKit | Logging/replay framework |
| PathPlanner | Autonomous path following |
| Phoenix 6 | CTRE motor control (TalonFX) |
| REVLib | REV hardware (SparkMax) |
| Elastic | Dashboard layout |
