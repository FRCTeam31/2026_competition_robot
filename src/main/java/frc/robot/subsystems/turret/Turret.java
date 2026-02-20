package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.prime.subsystems.LoggedSubsystem;
import org.prime.subsystems.turret.TurretUtilities;
import org.prime.util.MutVector;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Container;
import frc.robot.FieldTargets;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightCameraInputsAutoLogged;

import static org.prime.util.PhysicsConstants.GRAVITY;

/**
 * Turret subsystem responsible for aiming and shooting fuel into the hub.
 * Manages flywheel speed, yaw/pitch targeting, and the feeder uptake mechanism.
 * Uses {@link TurretUtilities} for projectile physics and sensor pose math.
 */
public class Turret extends LoggedSubsystem {
    private ITurret _turret;

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
        /** Ballistics solution found — a valid aim vector exists. */
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

    // CTRE Control Requests
    private final VelocityVoltage _flywheelControl = new VelocityVoltage(0);
    private final MotionMagicVoltage _yawControl = new MotionMagicVoltage(0);
    private final DutyCycleOut _yawManualControl = new DutyCycleOut(0);
    private double _manualFlywheelVelocityRPS;
    private double _manualYawInput;

    // Mutable Vectors
    private final MutVector _mutNominalTargetVector = new MutVector();
    private final MutVector _mutRobotVelocityVector = new MutVector();
    private final MutVector _mutTurretTangentVelocityVector = new MutVector();

    // Manual Control Suppliers
    private DoubleSupplier _yawSupplier;
    private DoubleSupplier _pitchSupplier;

    // Yaw setpoint filter to smooth out noise from pose estimation
    private final LinearFilter _yawFilter = LinearFilter.singlePoleIIR(0.2, Robot.defaultPeriodSecs);

    /**
     * Constructs the Turret subsystem, selecting the real or simulated IO layer
     * based on the current runtime environment.
     */
    public Turret() {
        setName("Turret");
        _turret = Robot.isReal()
                ? new TurretReal()
                : new TurretSim();
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
        if (inputs.TargetingState == TargetingState.AUTO) {
            var turretPose = getTurretPose();
            var target = resolveAutoTarget(turretPose);

            calculateTurretVectorFromRobotPose(target, turretPose);
        }

        actOnTargetingState(inputs.TargetingState, _mutNominalTargetVector);
        actOnFlywheelState(inputs.FlywheelState, inputs.TargetingState, _mutNominalTargetVector);
        actOnFeedState(inputs.FeedState);
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
        var target = FieldTargets.GetPassingPosition(SuperStructure.Swerve.EstimatedRobotPose);
        if (target == null) {
            // We are not in the neutral zone, target the hub.
            target = FieldTargets.GetHubPosition();
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
        double deltaX = target.getX() - initialX;
        double deltaY = target.getY() - initialY;
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
        return target;
    }

    /**
     * Corrects the turret's yaw position based on the horizontal offset from the limelight target.
     * @param limelightInputs
     * @return true if correction was applied, false otherwise
     */
    private boolean correctTurretYaw(LimelightCameraInputsAutoLogged limelightInputs) {
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

        _turret.controlYaw(_yawControl.withPosition(correctedPositionRotations));

        return true;
    }

    /**
     * Controls turret yaw and hood based on the current targeting state.
     * 
     * @param targetingState The current targeting mode (MANUAL, AUTO, or STOPPED)
     * @param aimVector The calculated aim vector (only used in AUTO mode, may be null otherwise)
     */
    private void actOnTargetingState(TargetingState targetingState, MutVector aimVector) {
        switch (targetingState) {
            // TODO: Implement pitch control once CAD finalizes turret
            case MANUAL:
                _turret.controlYaw(
                        _yawManualControl.withOutput(TurretMap.YAW_MAX_MANUAL_SPEED * _yawSupplier.getAsDouble()));

                // TODO: Limit hood motion based on current angle and max/min angle
                _turret.controlHood(TurretMap.PITCH_MAX_MANUAL_SPEED * _pitchSupplier.getAsDouble()); // <hood pitch implementation>
                break;
            case AUTO:
                var limelightInputs = SuperStructure.VisionLimelights.get(VisionMap.LimelightTurretName);
                boolean correctionApplied = TurretMap.UPDATE_LIMELIGHT_POSE && correctTurretYaw(limelightInputs);

                // Calculate the yaw if no correction was applied
                if (!correctionApplied) {
                    // aimVector.getYaw() is field-relative (degrees), but the turret motor
                    // position is robot-relative. Subtract the robot's heading to convert.
                    var fieldYawDeg = aimVector.getYaw();
                    fieldYawDeg += _manualYawInput * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;

                    var robotHeadingDeg = SuperStructure.Swerve.EstimatedRobotPose.getRotation().getDegrees();
                    var robotRelativeYawRotations = _yawFilter.calculate((fieldYawDeg - robotHeadingDeg) / 360.0);

                    _turret.controlYaw(_yawControl.withPosition(robotRelativeYawRotations));
                }

                // var pitch = aimVector.getPitch();
                // TODO: complete hood pitch implementation
                break;
            case STOPPED:
            default:
                _turret.controlFlywheel(_flywheelControl.withVelocity(0));
                _turret.controlYaw(_yawManualControl.withOutput(0));
                break;
        }
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
                _turret.controlFlywheel(_flywheelControl.withVelocity(TurretMap.FLYWHEEL_IDLE_VELOCITY_RPS));
                break;
            case STOPPED:
                _turret.controlFlywheel(_flywheelControl.withVelocity(0));
                break;
            case SHOOTING:
                if (targetingState == TargetingState.MANUAL) {
                    _turret.controlFlywheel(_flywheelControl.withVelocity(_manualFlywheelVelocityRPS));
                } else {
                    // Temp relation between flywheel speed and fuel velocity, will be replaced
                    // with a more concrete relation after testing
                    var targetVelocity = aimVector.getMagnitude();
                    var targetFlywheelOmega = (targetVelocity * (7 / 2)) / TurretMap.FLYWHEEL_RADIUS;
                    _turret.controlFlywheel(_flywheelControl.withVelocity(targetFlywheelOmega));
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
                _turret.setFeederSpeed(.5);
                break;
            case REVERSED:
                _turret.setFeederSpeed(-.5);
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
}
