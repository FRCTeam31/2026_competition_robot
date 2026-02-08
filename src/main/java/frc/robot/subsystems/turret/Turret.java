package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.prime.util.MutVector;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldTargets;
import frc.robot.SuperStructure;

public class Turret extends SubsystemBase {
    private ITurret _turret;

    public enum FlywheelState {
        IDLE,
        SHOOTING,
        STOPPED
    }

    public enum TargetingState {
        MANUAL,
        AUTO,
        STOPPED
    }

    public enum LockOnState {
        SHOT_CALCULATED,
        SHOT_NOT_CALCULATED,
        YAW_LOCKED_ON,
        YAW_NOT_LOCKED_ON,
        FLYWHEEL_AT_SPEED,
        FLYWHEEL_NOT_AT_SPEED
    }

    public enum UptakeState {
        FORWARDS,
        REVERSED,
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

    public Turret(boolean isReal) {
        setName("Turret");
        _turret = isReal ? new TurretReal() : new TurretSim();
    }

    public MutVector calculateTurretVectorFromRobotPose(Pose3d targetPose) {
        var robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);

        var targetDistance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        if (targetDistance < TurretMap.MIN_SHOT_DISTANCE_METERS) {
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_NOT_CALCULATED;
            return _mutNominalTargetVector;
        }

        try {
            _mutNominalTargetVector.setToTargetVector(robotPose,
                    targetPose,
                    TurretMap.HOOD_MIN_ANGLE_DEGREES,
                    TurretMap.HOOD_MAX_ANGLE_DEGREES,
                    TurretMap.FLYWHEEL_MIN_SPEED,
                    TurretMap.FLYWHEEL_MAX_SPEED);
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_CALCULATED;

            if (TurretMap.USE_SPEED_INTERPOLATION) {
                var interpolatedFlywheelSpeed = TurretMap.DISTANCE_TO_FLYWHEEL_SPEED_MAP.get(targetDistance);
                _mutNominalTargetVector.setMagnitude(interpolatedFlywheelSpeed);
            }

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

                return _mutNominalTargetVector
                        .minus(_mutRobotVelocityVector.plus(_mutTurretTangentVelocityVector));
            } else {
                return _mutNominalTargetVector;
            }
        } catch (Exception e) {
            SuperStructure.Turret.ShotCalculationState = LockOnState.SHOT_NOT_CALCULATED;
            return _mutNominalTargetVector;
        }
    }

    public void setYawSupplier(DoubleSupplier supplier) {
        _yawSupplier = supplier;
    }

    public void setPitchSupplier(DoubleSupplier supplier) {
        _pitchSupplier = supplier;
    }

    private void actOnState(TurretInputsAutoLogged inputs) {
        var target = Pose3d.kZero;
        MutVector aimVector = null;

        if (inputs.TargetingState == TargetingState.AUTO) {
            target = FieldTargets.GetPassingPosition(SuperStructure.Swerve.EstimatedRobotPose);
            if (target == null) {
                // We are not in the neutral zone, target the hub.
                target = FieldTargets.GetHubPosition();
            }
            aimVector = calculateTurretVectorFromRobotPose(target);
        }

        // TODO: explain
        switch (inputs.TargetingState) {
            // TODO: Implement pitch control once CAD finalizes turret
            case MANUAL:
                _turret.controlYaw(
                        _yawManualControl.withOutput(TurretMap.YAW_MAX_MANUAL_SPEED * _yawSupplier.getAsDouble()));

                // TODO: Limit hood motion based on current angle and max/min angle
                _turret.controlHood(TurretMap.PITCH_MAX_MANUAL_SPEED * _pitchSupplier.getAsDouble()); // <hood pitch implementation>
                break;
            case AUTO:
                var yaw = aimVector.getYaw();
                yaw += _manualYawInput * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;
                _turret.controlYaw(_yawControl.withPosition(yaw));

                var pitch = aimVector.getPitch();
                // <hood pitch implementation>
                break;
            case STOPPED:
            default:
                _turret.controlFlywheel(_flywheelControl.withVelocity(0));
                _turret.controlYaw(_yawManualControl.withOutput(0));
                break;
        }

        switch (inputs.FlywheelState) {
            case IDLE:
                // Low speed for flywheel
                _turret.controlFlywheel(_flywheelControl.withVelocity(TurretMap.FLYWHEEL_IDLE_VELOCITY_RPS));
                break;
            case STOPPED:
                _turret.controlFlywheel(_flywheelControl.withVelocity(0));
                break;
            case SHOOTING:
                if (inputs.TargetingState == TargetingState.MANUAL) {
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

        // Turret Feed States
        switch (inputs.FeedState) {
            case FORWARDS:
                _turret.setFeederSpeed(.5);
                break;
            case REVERSED:
                _turret.setFeederSpeed(-.5);
            case STOPPED:
            default:
                _turret.setFeederSpeed(0);
        }
    }

    /**
     * Calculates the limelight's 3D pose (position and rotation) relative to the robot's centerpoint on the ground.
     * 
     * This method accounts for:
     * - The turret's rotation origin offset from the robot center
     * - The current turret rotation angle
     * - The limelight's fixed offset from the turret rotation center
     * - The limelight's fixed POV angle relative to the turret
     * 
     * @return Pose3d representing the limelight's position (XYZ) and rotation (pitch, yaw, roll) 
     *         from the robot's centerpoint on the ground
     */
    public Pose3d getLimelightPose3dFromRobotCenter() {
        // Get current turret rotation from inputs
        double turretRotationRadians = SuperStructure.Turret.TurretRotation.getRadians();

        // Step 1: Calculate turret rotation center position from robot center
        Translation3d turretCenterFromRobotCenter = new Translation3d(
                TurretMap.TURRET_CENTER_OFFSET_X,
                TurretMap.TURRET_CENTER_OFFSET_Y,
                TurretMap.TURRET_CENTER_OFFSET_Z);

        // Step 2: Calculate limelight offset from turret center, accounting for turret rotation
        // The limelight offset rotates with the turret around the Z-axis (yaw)
        double rotatedLimelightX = TurretMap.LIMELIGHT_OFFSET_X * Math.cos(turretRotationRadians)
                - TurretMap.LIMELIGHT_OFFSET_Y * Math.sin(turretRotationRadians);
        double rotatedLimelightY = TurretMap.LIMELIGHT_OFFSET_X * Math.sin(turretRotationRadians)
                + TurretMap.LIMELIGHT_OFFSET_Y * Math.cos(turretRotationRadians);

        Translation3d limelightOffsetFromTurretCenter = new Translation3d(
                rotatedLimelightX,
                rotatedLimelightY,
                TurretMap.LIMELIGHT_OFFSET_Z // Z offset doesn't change with rotation
        );

        // Step 3: Combine to get total limelight position from robot center
        Translation3d limelightPositionFromRobotCenter = turretCenterFromRobotCenter
                .plus(limelightOffsetFromTurretCenter);

        // Step 4: Calculate limelight rotation
        // The limelight's rotation includes both its fixed POV angle and the turret's rotation
        Rotation3d limelightRotation = new Rotation3d(
                TurretMap.LIMELIGHT_ROLL, // Roll (around X-axis)
                TurretMap.LIMELIGHT_PITCH, // Pitch (around Y-axis)
                TurretMap.LIMELIGHT_YAW + turretRotationRadians // Yaw (around Z-axis) - includes turret rotation
        );

        // Step 5: Create and return the final Pose3d
        return new Pose3d(limelightPositionFromRobotCenter, limelightRotation);
    }

    @Override
    public void periodic() {
        _turret.updateInputs(SuperStructure.Turret);
        // TODO: Currently in field relative, possibly change later
        SuperStructure.Turret.targetVector = _mutNominalTargetVector.getTranslation3d()
                .plus(new Translation3d(SuperStructure.Swerve.EstimatedRobotPose.getTranslation()));

        Logger.processInputs(getName(), SuperStructure.Turret);

        actOnState(SuperStructure.Turret);
    }

    public Command setFlywheel(FlywheelState state) {
        return this.runOnce(() -> SuperStructure.Turret.FlywheelState = state);
    }

    public Command setFlywheelManualSpeedRps(double rps) {
        return this.runOnce(() -> _manualFlywheelVelocityRPS = rps);
    }

    public Command setTargeting(TargetingState state) {
        return this.runOnce(() -> SuperStructure.Turret.TargetingState = state);
    }

    /**
     * Sets the manual yaw input for turret control. This input is used as a trim multiplier in AUTO targeting mode and as direct control input in MANUAL targeting mode.
     * @param input
     * @return
     */
    public Command setTargetingManualYawInput(double input) {
        return this.runOnce(() -> _manualYawInput = input);
    }

    public Command setFeed(UptakeState state) {
        return this.runOnce(() -> SuperStructure.Turret.FeedState = state);
    }
}
