package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.prime.subsystems.LoggedSubsystem;
import org.prime.sysid.SysIdRoutineHelper;
import org.prime.subsystems.turret.TurretDeadZoneHelper;
import org.prime.subsystems.turret.TurretUtilities;
import org.prime.util.IDWController;
import org.prime.util.MutVector;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Container;
import frc.robot.FieldTargets;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import frc.robot.subsystems.vision.VisionMap;

import frc.robot.FieldTargets.TargetType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.prime.util.PhysicsConstants.GRAVITY;

/**
 * Turret subsystem responsible for aiming and shooting fuel into the hub.
 * Manages flywheel speed, yaw/pitch targeting, and the feeder uptake mechanism.
 * Uses {@link TurretUtilities} for projectile physics and sensor pose math.
 */
public class Turret extends LoggedSubsystem {
    private ITurret _turret;
    private final TurretDeadZoneHelper _deadZoneHelper;

    /** Represents the desired operational state of the flywheel. */
    public enum FlywheelState {
        /** Flywheel spinning at a low idle speed, ready to ramp up quickly. */
        IDLE,
        /** Flywheel spinning at the calculated or manual target speed for shooting. */
        SHOOTING,
        /** Flywheel completely stopped (zero velocity). */
        STOPPED
    }

    /** Represents the turret's yaw and pitch aiming mode. */
    public enum TargetingState {
        /** Operator controls yaw and pitch directly via joystick suppliers. */
        MANUAL,
        /** Turret automatically aims at the resolved field target using ballistics. */
        AUTO,
        /** All targeting outputs zeroed; turret holds position. */
        STOPPED
    }

    /**
     * Tracks the progress of the turret's shot readiness pipeline.
     * Each value represents one stage of the lock-on sequence.
     */
    public enum LockOnState {
        /** Ballistics solution found -- a valid aim vector exists. */
        SHOT_CALCULATED,
        /** No valid ballistics solution (target too far, degenerate geometry, etc.). */
        SHOT_NOT_CALCULATED,
        /** Turret yaw is within tolerance of the calculated setpoint. */
        YAW_LOCKED_ON,
        /** Turret yaw has not yet converged on the calculated setpoint. */
        YAW_NOT_LOCKED_ON,
        /** Flywheel velocity is within tolerance of the target speed. */
        FLYWHEEL_AT_SPEED,
        /** Flywheel velocity has not yet reached the target speed. */
        FLYWHEEL_NOT_AT_SPEED
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

    // Manual Control
    private double _manualFlywheelVelocityRPS;
    private double _manualYawInput;

    // IDW Controller
    private final IDWController _flywheelController = new IDWController(TurretMap.FLYWHEEL_IDW_ENTRIES, 2);

    // Mutable Vectors
    private final MutVector _mutNominalTargetVector = new MutVector();
    private final MutVector _mutRobotVelocityVector = new MutVector();
    private final MutVector _mutTurretTangentVelocityVector = new MutVector();

    // Manual Control Suppliers
    private DoubleSupplier _yawSupplier;
    private DoubleSupplier _pitchSupplier;

    // Yaw setpoint filter to smooth out noise from pose estimation
    private final LinearFilter _yawFilter = LinearFilter.singlePoleIIR(0.2, Robot.defaultPeriodSecs);

    // SysId characterization routines
    private final SysIdRoutineHelper _flywheelSysId;
    private final SysIdRoutineHelper _yawSysId;

    private final double maxFeedInwardsPercentOut = (TurretMap.FEEDER_INVERTED ? -1 : 1)
            * TurretMap.MAX_FEED_PERCENT_OUT;

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

        // Configure SysId routine for flywheel characterization
        _flywheelSysId = new SysIdRoutineHelper(
                this,
                "TurretFlywheel",
                (voltage) -> _turret.setFlywheelVoltage(voltage.in(Units.Volts)),
                (log) -> log.motor("flywheel")
                        .voltage(Units.Volts.of(SuperStructure.Turret.FlywheelVoltage))
                        .angularVelocity(SuperStructure.Turret.FlywheelVelocity));

        // Configure SysId routine for turret yaw characterization
        _yawSysId = new SysIdRoutineHelper(
                this,
                "TurretYaw",
                (voltage) -> _turret.setYawVoltage(voltage.in(Units.Volts)),
                (log) -> log.motor("yaw")
                        .voltage(Units.Volts.of(SuperStructure.Turret.YawVoltage))
                        .angularPosition(Units.Rotations.of(
                                SuperStructure.Turret.TurretRotation.getRotations())));
    }

    /**
     * Calculates the aim vector required to hit the target from the turret's current pose.
     * Delegates projectile physics to {@link TurretUtilities#calculateAimVector} and optionally
     * applies speed interpolation and motion compensation. Updates
     * {@link SuperStructure.Turret#ShotCalculationState} to reflect whether a valid solution was found.
     *
     * @param targetPose The 3D pose of the field target to aim at
     * @param turretPose The 3D pose of the turret in field-relative coordinates
     */
    public void calculateTurretVectorFromRobotPose(Pose3d targetPose, Pose3d turretPose) {
        var robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);

        var targetDistance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        if (targetDistance < TurretMap.MIN_SHOT_DISTANCE_METERS) {
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_NOT_CALCULATED;
            return;
        }

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
                    TurretMap.FLYWHEEL_MIN_SPEED,
                    TurretMap.FLYWHEEL_MAX_SPEED);
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_CALCULATED;

            // TODO: Enable after empirical measurements correlating distance to required flywheel speed are taken
            if (TurretMap.USE_SPEED_INTERPOLATION) {
                var interpolatedFlywheelSpeed = TurretMap.DISTANCE_TO_FLYWHEEL_SPEED_MAP.get(targetDistance);
                _mutNominalTargetVector.setMagnitude(interpolatedFlywheelSpeed);
            }

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
        } catch (Exception e) {
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
     * Sets the supplier used to read manual yaw input from the operator controller.
     *
     * @param supplier A {@link DoubleSupplier} providing yaw input in the range [-1, 1]
     */
    public void setYawSupplier(DoubleSupplier supplier) {
        _yawSupplier = supplier;
    }

    /**
     * Sets the supplier used to read manual pitch/hood input from the operator controller.
     *
     * @param supplier A {@link DoubleSupplier} providing pitch input in the range [-1, 1]
     */
    public void setPitchSupplier(DoubleSupplier supplier) {
        _pitchSupplier = supplier;
    }

    /**
     * Master state dispatcher called each periodic cycle. Resolves the auto target
     * and aim vector when in AUTO mode, then delegates to the targeting, flywheel,
     * and feed state handlers.
     *
     * @param inputs The current turret inputs snapshot from {@link SuperStructure}
     */
    private void actOnState(TurretInputsAutoLogged inputs) {
        var turretPose = getTurretPose();

        // Override turret control if the robot is in a dead zone
        if (FieldTargets.InDeadZone(SuperStructure.Swerve.EstimatedRobotPose)) {
            _turret.controlHood(Angle.ofBaseUnits(0, Degrees));
            _turret.setFeederSpeed(0); // Set feed to 0
            actOnFlywheelState(inputs.FlywheelState, inputs.TargetingState, _mutNominalTargetVector); // Still Control flywheel
            resolveAutoTarget(turretPose); // Update logging with null target
        } else {
            if (inputs.TargetingState == TargetingState.AUTO) {
                var target = resolveAutoTarget(turretPose);

                calculateTurretVectorFromRobotPose(target, turretPose);
            }

            actOnTargetingState(inputs.TargetingState, _mutNominalTargetVector);
            actOnFlywheelState(inputs.FlywheelState, inputs.TargetingState, _mutNominalTargetVector);
            actOnFeedState(inputs.FeedState);
        }
    }

    /**
     * Resolves the target position for auto-aiming and logs the predicted projectile trajectory.
     * Uses the passing position if the robot is in the neutral zone, otherwise targets the hub.
     * The trajectory is computed using the current aim vector and projectile motion equations,
     * then recorded as an array of {@link Pose3d} for visualization in AdvantageScope.
     *
     * @param turretPose The turret's field-relative 3D pose, used as the trajectory origin
     * @return The resolved target {@link Pose3d} (hub or passing position)
     */
    private Pose3d resolveAutoTarget(Pose3d turretPose) {
        var target = FieldTargets.GetTargetPosition(SuperStructure.Swerve.EstimatedRobotPose);

        if (target.targetType() == TargetType.kDead) {
            recordOutput("Target Pose", (Pose3d) null);
            recordOutput("Optimal Fuel Trajectory", (Pose3d) null);
            return null;
        }

        recordOutput("Target Pose", target);

        double velocityX = _mutNominalTargetVector.getX();
        double velocityY = _mutNominalTargetVector.getY();
        double velocityZ = _mutNominalTargetVector.getZ();

        double initialX = turretPose.getX();
        double initialY = turretPose.getY();
        double initialZ = turretPose.getZ();

        // Calculate total flight time from horizontal distance and horizontal speed
        double horizontalSpeed = Math.hypot(velocityX, velocityY);
        double deltaX = target.targetPose().getX() - initialX;
        double deltaY = target.targetPose().getY() - initialY;
        double distance = Math.hypot(deltaX, deltaY);
        double totalTime = (horizontalSpeed > 1e-6) ? distance / horizontalSpeed : 0;

        var timeStep = 0.05; // seconds
        int numPoints = (int) (totalTime / timeStep) + 1;
        Pose3d[] trajectory = new Pose3d[numPoints];

        for (int i = 0; i < numPoints; i++) {
            double t = (numPoints > 1) ? totalTime * i / (numPoints - 1) : 0;

            // Projectile motion equations
            double x = initialX + velocityX * t;
            double y = initialY + velocityY * t;
            double z = initialZ + velocityZ * t - 0.5 * GRAVITY * t * t;

            // Calculate velocity direction for orientation
            double vx = velocityX;
            double vy = velocityY;
            double vz = velocityZ - GRAVITY * t;

            // Create rotation based on velocity direction
            double pitch = Math.atan2(vz, Math.sqrt(vx * vx + vy * vy));
            double yaw = Math.atan2(vy, vx);

            trajectory[i] = new Pose3d(
                    new Translation3d(x, y, z),
                    new Rotation3d(0, pitch, yaw));
        }

        recordOutput("Optimal Fuel Trajectory", trajectory);
        return target.targetPose();
    }

    /**
     * Corrects the turret's yaw position based on the horizontal offset from the limelight target.
     * @param limelightInputs
     * @return true if correction was applied, false otherwise
     */
    private boolean aimTurretYawUsingLimelight() {
        var limelightInputs = SuperStructure.VisionLimelights.get(VisionMap.LimelightTurretName);

        // Only correct when targeting the hub
        boolean isTargetingHub = FieldTargets.GetTargetPosition(SuperStructure.Swerve.EstimatedRobotPose)
                .targetType() == TargetType.kHub;
        if (!isTargetingHub) {
            return false;
        }

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

        _turret.controlYawAngle(Angle.ofBaseUnits(correctedPositionRotations, Rotations));

        return true;
    }

    /**
     * Controls turret yaw and hood based on the current targeting state.
     * When the dead zone is enabled, all yaw commands are routed through
     * {@link TurretDeadZoneHelper} so the turret never travels through
     * the forbidden arc.
     * 
     * @param targetingState The current targeting mode (MANUAL, AUTO, or STOPPED)
     * @param aimVector The calculated aim vector (only used in AUTO mode, may be null otherwise)
     */
    private void actOnTargetingState(TargetingState targetingState, MutVector aimVector) {
        switch (targetingState) {
            // TODO: Implement pitch control once CAD finalizes turret
            case MANUAL:
                var manualInput = TurretMap.YAW_MAX_MANUAL_PERCENT_OUT * _yawSupplier.getAsDouble();

                if (TurretMap.YAW_DEADZONE_ENABLED) {
                    // If the turret is in the dead zone and the input would drive it
                    // deeper in, block the input. Always allow rotating OUT.
                    if (_deadZoneHelper.shouldBlockManualInput(
                            SuperStructure.Turret.TurretRotation.getRotations(),
                            manualInput)) {
                        manualInput = 0;
                    }
                }

                _turret.setYawPercentOut(manualInput);

                // TODO: Limit hood motion based on current angle and max/min angle
                _turret.setHoodPercentOut(TurretMap.PITCH_MAX_MANUAL_PERCENT_OUT * _pitchSupplier.getAsDouble()); // <hood pitch implementation>
                break;
            case AUTO:
                // Calculate the yaw base on field position
                // Aim vector is field-relative
                var robotRelativeYawRotations = getRobotRelativeYawSetpoint(aimVector);

                // TODO: Use robot-relative field target estimate as a reference point and reject limelight input if it deviates too far from this.
                boolean correctionApplied = TurretMap.UPDATE_LIMELIGHT_POSE &&
                        aimTurretYawUsingLimelight();

                // Use robot-relative yaw estimate if no limelight correction was applied
                if (!correctionApplied) {
                    _turret.controlYawAngle(Angle.ofBaseUnits(robotRelativeYawRotations, Rotations));
                }

                // Aim hood based on the pitch angle from the aim vector
                var pitch = aimVector.getPitch();
                _turret.controlHood(Angle.ofBaseUnits(pitch, Degrees));
                break;
            case STOPPED:
            default:
                _turret.controlFlywheel(AngularVelocity.ofBaseUnits(0, RotationsPerSecond));
                _turret.controlYawAngle(Angle.ofBaseUnits(0, Rotations));
                break;
        }
    }

    private double getRobotRelativeYawSetpoint(MutVector aimVector) {
        // aimVector.getYaw() is field-relative (degrees), but the turret motor
        // position is robot-relative. Subtract the robot's heading to convert.
        var fieldYawDeg = aimVector.getYaw();
        fieldYawDeg += _manualYawInput * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;

        var robotHeadingDeg = SuperStructure.Swerve.EstimatedRobotPose.getRotation().getDegrees();
        var robotRelativeYawRotations = _yawFilter.calculate((fieldYawDeg - robotHeadingDeg) / 360.0);

        if (TurretMap.YAW_DEADZONE_ENABLED) {
            // Remap the desired setpoint so the turret never crosses the dead zone.
            robotRelativeYawRotations = _deadZoneHelper.computeLegalSetpoint(
                    SuperStructure.Turret.TurretRotation.getRotations(),
                    robotRelativeYawRotations);
        }

        return robotRelativeYawRotations;
    }

    /**
     * Controls the flywheel based on the current flywheel state and targeting mode.
     * 
     * @param flywheelState The desired flywheel behavior (IDLE, SHOOTING, or STOPPED)
     * @param targetingState The current targeting mode, used to determine manual vs auto speed
     * @param aimVector The calculated aim vector for deriving auto flywheel speed (may be null)
     */
    private void actOnFlywheelState(FlywheelState flywheelState, TargetingState targetingState, MutVector aimVector) {
        switch (flywheelState) {
            case IDLE:
                _turret.controlFlywheel(TurretMap.FLYWHEEL_IDLE_VELOCITY);
                break;
            case STOPPED:
                _turret.controlFlywheel(AngularVelocity.ofBaseUnits(0, RotationsPerSecond));
                break;
            case SHOOTING:
                if (targetingState == TargetingState.MANUAL) {
                    _turret.controlFlywheel(
                            AngularVelocity.ofBaseUnits(_manualFlywheelVelocityRPS, RotationsPerSecond));
                } else {
                    var targetVelocity = aimVector.getMagnitude();
                    // Interpolate the flywheel velocity using the target velocity and hood angle
                    var targetFlywheelOmegaRotationsPerSecond = _flywheelController.calculate(
                            targetVelocity,
                            SuperStructure.Turret.HoodAngle.in(Degrees));
                    _turret.controlFlywheel(
                            AngularVelocity.ofBaseUnits(targetFlywheelOmegaRotationsPerSecond, RotationsPerSecond));
                }
                break;
        }
    }

    /**
     * Controls the turret feeder based on the current feed state.
     * 
     * @param feedState The desired feeder direction (FORWARDS, REVERSED, or STOPPED)
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

        actOnState(SuperStructure.Turret);
    }

    /**
     * Creates a command that sets the flywheel to the given state.
     *
     * @param state The desired {@link FlywheelState}
     * @return A command that updates {@link SuperStructure.Turret#FlywheelState} when scheduled
     */
    public Command setFlywheel(FlywheelState state) {
        return this.runOnce(() -> SuperStructure.Turret.FlywheelState = state);
    }

    /**
     * Creates a command that sets the flywheel to a specific manual speed.
     * Only takes effect when {@link TargetingState#MANUAL} is active.
     *
     * @param rps The desired flywheel velocity in rotations per second
     * @return A command that stores the manual flywheel velocity setpoint
     */
    public Command setFlywheelManualSpeedRps(double rps) {
        return this.runOnce(() -> _manualFlywheelVelocityRPS = rps);
    }

    /**
     * Creates a command that sets the turret's targeting mode.
     *
     * @param state The desired {@link TargetingState}
     * @return A command that updates {@link SuperStructure.Turret#TargetingState} when scheduled
     */
    public Command setTargeting(TargetingState state) {
        return this.runOnce(() -> SuperStructure.Turret.TargetingState = state);
    }

    /**
     * Creates a command that sets the manual yaw input for turret control.
     * In {@link TargetingState#AUTO} mode this acts as a trim multiplier (scaled by
     * {@link TurretMap#AUTO_AIM_YAW_TRIM_DEGREES}). In {@link TargetingState#MANUAL}
     * mode it is unused (yaw comes from the yaw supplier instead).
     *
     * @param input The yaw trim/offset value, typically in the range [-1, 1]
     * @return A command that stores the manual yaw input
     */
    public Command setTargetingManualYawInput(double input) {
        return this.runOnce(() -> _manualYawInput = input);
    }

    /**
     * Creates a command that sets the feeder/uptake mechanism state.
     *
     * @param state The desired {@link UptakeState}
     * @return A command that updates {@link SuperStructure.Turret#FeedState} when scheduled
     */
    public Command setFeed(UptakeState state) {
        return this.runOnce(() -> SuperStructure.Turret.FeedState = state);
    }

    /**
     * Returns a SysId characterization command for the turret flywheel.
     *
     * @param testType  QUASISTATIC (ramp) or DYNAMIC (step)
     * @param direction FORWARD or REVERSE
     * @return A command that runs the specified SysId test on the flywheel
     */
    public Command sysIdFlywheelCommand(SysIdRoutineHelper.TestType testType,
            SysIdRoutineHelper.TestDirection direction) {
        return _flywheelSysId.getCommand(testType, direction);
    }

    /**
     * Returns a SysId characterization command for the turret yaw rotation.
     *
     * @param testType  QUASISTATIC (ramp) or DYNAMIC (step)
     * @param direction FORWARD or REVERSE
     * @return A command that runs the specified SysId test on the turret yaw motor
     */
    public Command sysIdYawCommand(SysIdRoutineHelper.TestType testType,
            SysIdRoutineHelper.TestDirection direction) {
        return _yawSysId.getCommand(testType, direction);
    }
}
