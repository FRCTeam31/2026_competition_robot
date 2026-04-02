package frc.robot.subsystems.turret;

import org.prime.subsystems.LoggedSubsystem;
import org.prime.subsystems.turret.TurretDeadZoneHelper;
import org.prime.subsystems.turret.TurretUtilities;
import org.prime.util.MutVector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Container;
import frc.robot.FieldTargets;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import frc.robot.subsystems.vision.VisionMap;

import frc.robot.FieldTargets.TargetType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static org.prime.util.PhysicsConstants.GRAVITY;

import java.util.function.Supplier;

/**
 * Turret subsystem responsible for aiming and shooting fuel into the hub.
 * Supports two operating modes:
 * <ul>
 *   <li><b>AUTO</b> - Ballistics-based targeting. Holding the fire button seeks all
 *       setpoints, waits for lock-on, then feeds. Releasing returns to home.</li>
 *   <li><b>MANUAL</b> - Operator directly adjusts yaw, hood, and flywheel setpoints
 *       via incremental commands. Fire button simply runs the feed motors.</li>
 * </ul>
 */
public class Turret extends LoggedSubsystem {
    private ITurret _turret;
    private final TurretDeadZoneHelper _deadZoneHelper;

    /** The turret's high-level operating mode, toggled by the operator. */
    public enum OperatingMode {
        /** Ballistics-driven auto-aim with lock-on gating before feeding. */
        AUTO,
        /** Operator controls all setpoints directly; fire button feeds immediately. */
        MANUAL,

        STOPPED
    }

    /** Ephemeral firing state - active only while the fire button is held. */
    public enum FiringState {
        /** Not firing. In AUTO mode the turret returns to home position. */
        IDLE,
        /** Fire button held. In AUTO mode the turret seeks, locks on, then feeds. */
        FIRING
    }

    /**
     * Tracks the progress of the turret's shot readiness pipeline.
     */
    public enum LockOnState {
        /** Ballistics solution found - a valid aim vector exists. */
        SHOT_CALCULATED,
        /** No valid ballistics solution. */
        SHOT_NOT_CALCULATED
    }

    /** Represents the direction of the feeder/uptake mechanism. */
    public enum UptakeState {
        /** Feeder running forward to deliver fuel to the flywheel. */
        FORWARDS,
        /** Feeder running in reverse to eject fuel. */
        REVERSED,
        /** Feeder stopped. */
        STOPPED
    }

    private enum TargetingType {
        kPose,
        kLimelight
    }

    public double _testFluwheelSpeed = 0.0;
    public double _testtargetvelocity = 0;
    public double _testdistance = 0;

    // -------------------- Manual Setpoints --------------------------
    private double _manualFlywheelVelocityRPS = TurretMap.FLYWHEEL_IDLE_VELOCITY.in(RotationsPerSecond);
    private double _manualYawDegrees = TurretMap.YAW_HOME_DEGREES;
    private double _manualHoodDegrees = TurretMap.HOOD_HOME_DEGREES;

    // -------------------- Internal Tracking --------------------------
    private double _targetFlywheelVelocityRPS = 0;
    private double _manualYawInput;
    private boolean _isHomingHood = false;

    // IDW Controller
    // private final IDWController _flywheelController = new IDWController(TurretMap.FLYWHEEL_IDW_ENTRIES, 2);

    // Mutable Vectors
    private final MutVector _mutNominalTargetVector = new MutVector();
    private final MutVector _mutRobotVelocityVector = new MutVector();
    private final MutVector _mutTurretTangentVelocityVector = new MutVector();

    // Yaw setpoint filter to smooth out noise from pose estimation
    private final LinearFilter _yawFilter = LinearFilter.singlePoleIIR(0.2, Robot.defaultPeriodSecs);

    // Boolean Events
    private BooleanEvent _turretYawResetSwitchEvent;

    private final double maxFeedInwardsPercentOut = (TurretMap.FEEDER_INVERTED ? -1 : 1)
            * TurretMap.MAX_FEED_PERCENT_OUT;

    // Alerts
    private Alert _targetTooCloseAlert = new Alert("Turret - SHOT NOT CALCULATED - Target too close!",
            AlertType.kError);
    private Alert _exceptionWhileCalculatingShotAlert = new Alert(
            "Turret - SHOT NOT CALCULATED - Exception while calculating shot solution!", AlertType.kError);

    /**
     * Constructs the Turret subsystem, selecting the real or simulated IO layer
     * based on the current runtime environment.
     */
    public Turret() {
        setName("Turret");
        _turret = Robot.isReal()
                ? new TurretReal()
                : new TurretSim();

        _deadZoneHelper = new TurretDeadZoneHelper(
                TurretMap.DEADZONE_START_DEGREES,
                TurretMap.DEADZONE_END_DEGREES);

        _turretYawResetSwitchEvent = new BooleanEvent(Robot.EventLoop,
                () -> SuperStructure.Turret.TurretRotationResetSwitch)
                .debounce(0.1);

        _turretYawResetSwitchEvent.ifHigh(() -> _turret.setYawSensorPosition(TurretMap.YAW_RESET_ANGLE));
    }

    /**
     * Calculates the aim vector required to hit the target from the turret's current pose.
     * Delegates projectile physics to {@link TurretUtilities#calculateAimVector} and optionally
     * applies speed interpolation and motion compensation. Updates
     * {@link SuperStructure.TurretSimple#ShotCalculationState} to reflect whether a valid solution was found.
     *
     * @param targetPose The 3D pose of the field target to aim at
     * @param turretPose The 3D pose of the turret in field-relative coordinates
     */
    public void calculateTurretVectorFromRobotPose(Pose3d targetPose, Pose3d turretPose) {
        var robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);

        var targetDistance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        _testdistance = targetDistance;
        if (targetDistance < TurretMap.MIN_SHOT_DISTANCE_METERS) {
            _targetTooCloseAlert.set(true);
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_NOT_CALCULATED;
            return;
        }
        _targetTooCloseAlert.set(false);

        try {
            recordOutput("Turret Pose", turretPose);
            TurretUtilities.calculateAimVector(
                    _mutNominalTargetVector,
                    turretPose,
                    targetPose,
                    TurretMap.TURRET_HEIGHT_ABOVE_GROUND,
                    TurretMap.HUB_OVERSHOOT_HEIGHT,
                    TurretMap.HOOD_MIN_ANGLE_DEGREES,
                    TurretMap.HOOD_MAX_ANGLE_DEGREES,
                    TurretMap.FLYWHEEL_MIN_SPEED_RPS,
                    TurretMap.FLYWHEEL_MAX_SPEED_RPS);
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_CALCULATED;

            // TODO: Enable after empirical measurements correlating distance to required flywheel speed are taken
            // if (TurretMap.USE_SPEED_INTERPOLATION) {
            //     var interpolatedFlywheelSpeed = TurretMap.DISTANCE_TO_FLYWHEEL_SPEED_MAP.get(targetDistance);
            //     _mutNominalTargetVector.setMagnitude(interpolatedFlywheelSpeed);
            // }

            // TODO: Refine motion compensation, enable, and test
            if (TurretMap.AUTO_MOTION_COMPENSATION) {
                var shotTimeToTarget = _mutNominalTargetVector.getTimeToTarget(targetDistance);
                ChassisSpeeds chassisSpeeds = SuperStructure.Swerve.RobotRelativeChassisSpeeds;

                _mutRobotVelocityVector.setCartesian(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        0);

                _mutTurretTangentVelocityVector.setPolar(
                        chassisSpeeds.omegaRadiansPerSecond * TurretMap.TURRET_DISTANCE_FROM_ROBOT_CENTER,
                        0,
                        TurretMap.TURRET_ROTATION_FROM_ROBOT_CENTER_TANGENT.getDegrees());

                _mutNominalTargetVector
                        .minus(_mutRobotVelocityVector.plus(_mutTurretTangentVelocityVector));
            }

            _exceptionWhileCalculatingShotAlert.set(false);
        } catch (Exception e) {
            _exceptionWhileCalculatingShotAlert.set(true);
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_NOT_CALCULATED;
        }
    }

    /**
    * Computes the turret's field-relative 3D pose by applying its mounting offset
    * and current rotation to the robot's estimated pose.
    *
    * @return The turret's pose in field coordinates
    */
    private Pose3d getTurretPose() {
        var robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);

        var turretTransform = new Transform3d(
                TurretMap.TURRET_ROBOT_ORIGIN,
                new Rotation3d(SuperStructure.Turret.TurretRotation));

        return robotPose.plus(turretTransform);
    }

    /**
     * Master state dispatcher called each periodic cycle. Routes to either
     * AUTO or MANUAL mode logic based on the current operating mode.
     *
     * @param inputs The current turret inputs snapshot from {@link SuperStructure}
     */
    private void actOnState(TurretInputsAutoLogged inputs) {
        // var startTime = Timer.getFPGATimestamp();
        switch (inputs.OperatingMode) {
            case AUTO:
                actOnAutoMode(inputs);
                // System.out.println(
                //         "Turret/periodic/actOnAutoMode - " + ((Timer.getFPGATimestamp() - startTime) * 1000) + "ms");
                // startTime = Timer.getFPGATimestamp();
                break;
            case MANUAL:
                actOnManualMode(inputs);
                // System.out.println(
                //         "Turret/periodic/actOnManualMode - " + ((Timer.getFPGATimestamp() - startTime) * 1000) + "ms");
                // startTime = Timer.getFPGATimestamp();
                break;
            case STOPPED:
                break;
        }

        // if (DriverStation.isEnabled()) {
        //     TurretLEDHelper.updateLEDs(inputs);
        // }

        actOnFeedState(inputs.FeedState);
        // System.out
        //         .println("Turret/periodic/actOnFeedState - " + ((Timer.getFPGATimestamp() - startTime) * 1000) + "ms");
    }

    // =========================== AUTO MODE ============================

    /**
     * AUTO mode logic. When FIRING:
     * <ol>
     *   <li>Resolve the field target and compute the ballistics solution</li>
     *   <li>Seek yaw, hood, and flywheel to calculated setpoints</li>
     *   <li>Once all three are on-target, start feeding fuel</li>
     * </ol>
     * When IDLE: return turret to home position, flywheel to idle speed, stop feeding.
     */
    private void actOnAutoMode(TurretInputsAutoLogged inputs) {
        var turretPose = getTurretPose();
        var target = resolveAutoTarget(turretPose); // Keep logging updated even when not firing

        //if (inputs.FiringState == FiringState.FIRING) {
        // Step 1: Resolve target
        if (target == null) {
            _turret.controlHood(Degrees.of(TurretMap.HOOD_MAX_ANGLE_DEGREES));
            return;
        }

        if (target.type == TargetingType.kLimelight) {
            turretPose = Pose3d.kZero;
        }

        // Step 2: Calculate ballistics and seek setpoints
        calculateTurretVectorFromRobotPose(target.pose, turretPose);

        // Yaw
        var robotRelativeYawRotations = getRobotRelativeYawSetpoint(_mutNominalTargetVector);
        boolean limelightCorrectionApplied = false;
        if (TurretMap.USE_LIMELIGHT_YAW_CORRECTION) {
            limelightCorrectionApplied = aimTurretYawUsingLimelight();
        }
        if (!limelightCorrectionApplied) {
            _turret.controlYawAngle(Rotations.of(robotRelativeYawRotations));
        }

        // Hood
        var pitch = _mutNominalTargetVector.getPitch();
        _turret.controlHood(Degrees.of(pitch));
        _testtargetvelocity = _mutNominalTargetVector.getMagnitude();
        // Flywheel
        if (inputs.FiringState == FiringState.FIRING) {
            // var targetVelocity = _mutNominalTargetVector.getMagnitude();
            // _targetFlywheelVelocityRPS = _flywheelController.calculate(
            //      targetVelocity, SuperStructure.Turret.HoodAngle.in(Degrees));

            // Calculate flywheel speed from target velocity
            // Made for use with a fixed hood
            var targetVelocityMPS = _mutNominalTargetVector.getMagnitude();
            _turret.controlFlywheel(RadiansPerSecond.of(targetVelocityMPS / TurretMap.FLYWHEEL_RADIUS));

            // var calculatedFlywheelSpeed = (double)TurretMap.TARGET_VELOCITY_TO_FLYHEEL_SPEED_MAP.get(targetVelocity);
            // _turrcet.controlFlywheel(RPM.of(calculatedFlywheelSpeed));

            // // Used for testing
            // _turret.controlFlywheel(RPM.of(_testFluwheelSpeed));
        }

        // Step 3: Once locked on, feed
        boolean allOnTarget = inputs.FlywheelAtTargetSpeed && inputs.YawOnTarget && inputs.HoodOnTarget;
        if (allOnTarget) {
            actOnFeedState(inputs.FeedState);
        } else {
            _turret.setFeederSpeed(0);
        }
        //} else {
        // IDLE - return to home, flywheel to idle, stop feed
        //goToHomePosition();
        //}
    }

    /**
     * Sends the turret to its home position: yaw to 180-degrees, hood fully up (lowered),
     * flywheel to idle speed, feed stopped.
     */
    private void goToHomePosition() {
        var homeYawRotations = TurretMap.YAW_HOME_DEGREES / 360.0;
        if (TurretMap.YAW_DEADZONE_ENABLED) {
            homeYawRotations = _deadZoneHelper.computeLegalSetpoint(
                    SuperStructure.Turret.TurretRotation.getRotations(), homeYawRotations);
        }
        _turret.controlYawAngle(Rotations.of(homeYawRotations));
        // _turret.controlHood(Degrees.of(TurretMap.HOOD_HOME_DEGREES));

        _targetFlywheelVelocityRPS = TurretMap.FLYWHEEL_IDLE_VELOCITY.in(RotationsPerSecond);
        _turret.controlFlywheel(RotationsPerSecond.of(_targetFlywheelVelocityRPS));

        _turret.setFeederSpeed(0);
    }

    // =========================== MANUAL MODE ==============================

    /**
     * MANUAL mode logic. The operator's stored setpoints (yaw, hood, flywheel)
     * are always applied. When FIRING, the feed motors run immediately
     * (no lock-on gating). When IDLE, feed is stopped.
     */
    private void actOnManualMode(TurretInputsAutoLogged inputs) {
        // Apply manual yaw setpoint (respecting dead zone)
        var manualYawRotations = Rotation2d.fromDegrees(_manualYawDegrees).getRotations();
        if (TurretMap.YAW_DEADZONE_ENABLED) {
            manualYawRotations = _deadZoneHelper.computeLegalSetpoint(
                    SuperStructure.Turret.TurretRotation.getRotations(), manualYawRotations);
        }
        SuperStructure.Turret.DesiredTurretRotationDegrees = Rotation2d.fromRotations(manualYawRotations).getDegrees();
        _turret.controlYawAngle(Rotations.mutable(manualYawRotations));

        // Apply manual hood setpoint
        // _turret.controlHood(Degrees.of(_manualHoodDegrees));

        // Apply manual flywheel speed
        _targetFlywheelVelocityRPS = _manualFlywheelVelocityRPS;
        _turret.controlFlywheel(RotationsPerSecond.mutable(_targetFlywheelVelocityRPS));

        // Feed: run immediately when firing, stop when idle
        if (inputs.FiringState == FiringState.FIRING) {
            actOnFeedState(inputs.FeedState);
        } else {
            _turret.setFeederSpeed(0);
        }

        actOnFeedState(inputs.FeedState);
    }

    // =========================== SHARED HELPERS ===========================

    private record TargetingData(Pose3d pose, TargetingType type) {
    }

    /**
     * Resolves the target position for auto-aiming and logs the predicted projectile trajectory.
     *
     * @param turretPose The turret's field-relative 3D pose, used as the trajectory origin
     * @return The resolved target {@link Pose3d}, or null if in a dead zone
     */
    private TargetingData resolveAutoTarget(Pose3d turretPose) {
        var target = FieldTargets.GetTargetPosition(SuperStructure.Swerve.EstimatedRobotPose);
        var targetType = TargetingType.kPose;

        if (target.targetType() == TargetType.kDead) {
            recordOutput("Target Pose", (Pose3d) null);
            recordOutput("Optimal Fuel Trajectory", (Pose3d) null);
            return null;
        }

        if (TurretMap.USE_LIMELIGHT_TARGETING) {
            targetType = TargetingType.kLimelight;
            var limelightInputs = SuperStructure.VisionLimelights.get(VisionMap.LimelightTurretName);
            target = FieldTargets.GetHubTargetFromTag(
                    limelightInputs.CurrentResults.targets_Fiducials[0].fiducialID,
                    limelightInputs.TagPoseRobotSpace);
        }

        recordOutput("Target Pose", target);

        double velocityX = _mutNominalTargetVector.getX();
        double velocityY = _mutNominalTargetVector.getY();
        double velocityZ = _mutNominalTargetVector.getZ();

        double initialX = turretPose.getX();
        double initialY = turretPose.getY();
        double initialZ = turretPose.getZ();

        double horizontalSpeed = Math.hypot(velocityX, velocityY);
        double deltaX = target.targetPose().getX() - initialX;
        double deltaY = target.targetPose().getY() - initialY;
        double distance = Math.hypot(deltaX, deltaY);
        double totalTime = (horizontalSpeed > 1e-6) ? distance / horizontalSpeed : 0;

        var timeStep = 0.05;
        int numPoints = (int) (totalTime / timeStep) + 1;
        Pose3d[] trajectory = new Pose3d[numPoints];

        for (int i = 0; i < numPoints; i++) {
            double t = (numPoints > 1) ? totalTime * i / (numPoints - 1) : 0;
            double x = initialX + velocityX * t;
            double y = initialY + velocityY * t;
            double z = initialZ + velocityZ * t - 0.5 * GRAVITY * t * t;
            double vx = velocityX;
            double vy = velocityY;
            double vz = velocityZ - GRAVITY * t;
            double pitch = Math.atan2(vz, Math.sqrt(vx * vx + vy * vy));
            double yaw = Math.atan2(vy, vx);

            trajectory[i] = new Pose3d(
                    new Translation3d(x, y, z),
                    new Rotation3d(0, pitch, yaw));
        }

        recordOutput("Optimal Fuel Trajectory", trajectory);
        return new TargetingData(target.targetPose(), targetType);
    }

    /**
     * Corrects the turret's yaw position based on the horizontal offset from the limelight target.
     * @return true if correction was applied, false otherwise
     */
    private boolean aimTurretYawUsingLimelight() {
        var limelightInputs = SuperStructure.VisionLimelights.get(VisionMap.LimelightTurretName);

        boolean isTargetingHub = FieldTargets.GetTargetPosition(SuperStructure.Swerve.EstimatedRobotPose)
                .targetType() == TargetType.kHub;
        if (!isTargetingHub) {
            return false;
        }

        // var isTargetAHubCenter = ...
        if (limelightInputs.TargetHorizontalOffset == null) {
            return false;
        }

        var horizontalError = limelightInputs.TargetHorizontalOffset;
        if (Math.abs(horizontalError.getDegrees()) < TurretMap.TURRET_CORRECTION_THRESHOLD_DEGREES) {
            return false;
        }

        double errorRotations = horizontalError.getDegrees() / 360.0;
        double currentPositionRotations = SuperStructure.Turret.TurretRotation.getRotations();
        double correctedPositionRotations = _yawFilter.calculate(currentPositionRotations + errorRotations);

        _turret.controlYawAngle(Rotations.of(correctedPositionRotations));
        return true;
    }

    private double getRobotRelativeYawSetpoint(MutVector aimVector) {
        var fieldYawDeg = aimVector.getYaw();
        fieldYawDeg += _manualYawInput * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;

        var robotHeadingDeg = SuperStructure.Swerve.EstimatedRobotPose.getRotation().getDegrees();
        var robotRelativeYawRotations = _yawFilter.calculate((fieldYawDeg - robotHeadingDeg) / 360.0);

        if (TurretMap.YAW_DEADZONE_ENABLED) {
            robotRelativeYawRotations = _deadZoneHelper.computeLegalSetpoint(
                    SuperStructure.Turret.TurretRotation.getRotations(),
                    robotRelativeYawRotations);
        }

        return robotRelativeYawRotations;
    }

    /**
     * Controls the turret feeder based on the current feed state.
     */
    private void actOnFeedState(UptakeState feedState) {
        switch (feedState) {
            case FORWARDS:
                _turret.setFeederSpeed(maxFeedInwardsPercentOut);
                break;
            case REVERSED:
                _turret.setFeederSpeed(-maxFeedInwardsPercentOut);
                break;
            case STOPPED:
            default:
                _turret.setFeederSpeed(0);
                break;
        }
    }

    // =========================== PERIODIC =================================

    @Override
    public void periodic() {
        _turret.updateInputs(SuperStructure.Turret);

        if (TurretMap.UPDATE_LIMELIGHT_POSE) {
            var llPose = TurretUtilities.calculateSensorPose(
                    TurretMap.TURRET_ROBOT_ORIGIN,
                    TurretMap.LIMELIGHT_TRANSFORM_FROM_TURRET_CENTER,
                    SuperStructure.Turret.TurretRotation.getRadians());

            Container.LimelightVision.setCameraPose(VisionMap.LimelightTurretName, llPose);
        }

        processInputs(SuperStructure.Turret);

        if (!_isHomingHood) {
            actOnState(SuperStructure.Turret);
        }
    }

    // =========================== COMMAND FACTORIES ========================

    public Command setHoodAngle(Angle angle) {
        return Commands.runOnce(() -> {
            _turret.controlHood(angle);
        });
    }

    public Command setHoodAngleSupplier(Supplier<Angle> angle) {
        return Commands.runOnce(() -> {
            _turret.controlHood(angle.get());
        });
    }

    // --- Mode Toggle -------------------------------------------------

    /**
     * Toggles between AUTO and MANUAL operating modes.
     */
    public Command toggleOperatingMode() {
        return this.runOnce(() -> {
            if (SuperStructure.Turret.OperatingMode == OperatingMode.AUTO) {
                SuperStructure.Turret.OperatingMode = OperatingMode.MANUAL;
            } else {
                SuperStructure.Turret.OperatingMode = OperatingMode.AUTO;
                // Reset manual setpoints to home when switching back to auto
                _manualYawDegrees = TurretMap.YAW_HOME_DEGREES;
                _manualHoodDegrees = TurretMap.HOOD_HOME_DEGREES;
                _manualFlywheelVelocityRPS = TurretMap.FLYWHEEL_IDLE_VELOCITY.in(RotationsPerSecond);
            }
        });
    }

    /**
     * Sets the operating mode directly.
     */
    public Command setOperatingMode(OperatingMode mode) {
        return this.runOnce(() -> SuperStructure.Turret.OperatingMode = mode);
    }

    // --- Firing -------------------------------------------------------

    /**
     * Sets the firing state. Bind to fire button press/release.
     */
    public Command setFiring(FiringState state) {
        return this.runOnce(() -> SuperStructure.Turret.FiringState = state);
    }

    // --- Feed Direction ------------------------------------------------

    /**
     * Sets the feeder/uptake mechanism direction.
     */
    public Command setFeed(UptakeState state) {
        return this.runOnce(() -> SuperStructure.Turret.FeedState = state);
    }

    // --- Manual Setpoint Adjustments -----------------------------------

    /**
     * Adjusts the manual flywheel speed setpoint by {@code deltaRPS} rotations per second.
     * Clamped to [0, FLYWHEEL_MAX_SPEED].
     */
    public Command adjustManualFlywheelSpeed(double deltaRPS) {
        return setOperatingMode(OperatingMode.MANUAL).andThen(this.runOnce(() -> {
            _manualFlywheelVelocityRPS = MathUtil.clamp(
                    _manualFlywheelVelocityRPS + deltaRPS,
                    0, TurretMap.FLYWHEEL_MAX_SPEED_RPS);
        }));
    }

    /**
     * Adjusts the manual hood angle setpoint by {@code deltaDegrees}.
     * Clamped to [HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES].
     */
    public Command adjustManualHoodAngle(double deltaDegrees) {
        return setOperatingMode(OperatingMode.MANUAL).andThen(this.runOnce(() -> {
            _manualHoodDegrees = MathUtil.clamp(
                    _manualHoodDegrees + deltaDegrees,
                    TurretMap.HOOD_MIN_ANGLE_DEGREES, TurretMap.HOOD_MAX_ANGLE_DEGREES);
        }));
    }

    /**
     * Adjusts the manual yaw setpoint by {@code deltaDegrees}.
     * Clamped to [0, 360] and respects the dead zone.
     */
    public Command adjustManualYaw(double deltaDegrees) {
        return setOperatingMode(OperatingMode.MANUAL).andThen(this.runOnce(() -> {
            _manualYawDegrees = MathUtil.clamp(
                    _manualYawDegrees + deltaDegrees,
                    0, 360);
        }));
    }

    /**
     * Sets the manual yaw trim input for AUTO mode (acts as a trim multiplier).
     */
    public Command setAutoYawTrimInput(double input) {
        return this.runOnce(() -> _manualYawInput = input);
    }

    // --- Homing --------------------------------------------------------

    public Command homeTurretHood() {
        return Commands.sequence(
                Commands.runOnce(() -> _isHomingHood = true),
                Commands.run(() -> _turret.setHoodPercentOut(-0.1), this)
                        .withTimeout(1.0),
                Commands.runOnce(() -> {
                    _turret.setHoodSensorPosition(
                            Degrees.of(TurretMap.HOOD_MAX_ANGLE_DEGREES));
                    _turret.setHoodPercentOut(0);
                    _isHomingHood = false;
                })).finallyDo((interrupted) -> {
                    _turret.setHoodPercentOut(0);
                    _isHomingHood = false;
                });
    }
}
