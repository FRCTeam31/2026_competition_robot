package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import org.prime.util.MutVector;
import org.prime.util.PhysicsConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose3d;
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

    // CTRE Control Requests
    private final VelocityVoltage _flywheelControl = new VelocityVoltage(0);
    private final MotionMagicVoltage _yawControl = new MotionMagicVoltage(0);
    private final DutyCycleOut _yawManualControl = new DutyCycleOut(0);
    private double _manualFlywheelVelocityRPS;
    private double _manualYawSpeed;

    // Mutable Vectors
    private final MutVector _mutNominalTargetVector = new MutVector();
    private final MutVector _mutRobotVelocityVector = new MutVector();
    private final MutVector _mutTurretTangentVelocityVector = new MutVector();

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
            case MANUAL:
                _turret.controlYaw(_yawManualControl.withOutput(_manualYawSpeed));
                break;
            case AUTO:
                var yaw = aimVector.getYaw();
                yaw += _manualYawSpeed * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;
                _turret.controlYaw(_yawControl.withPosition(yaw));

                // TODO: Implement pitch control once CAD finalizes turret
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
    }

    @Override
    public void periodic() {
        _turret.updateInputs(SuperStructure.Turret);
        Logger.processInputs(getName(), SuperStructure.Turret);

        actOnState(SuperStructure.Turret);
    }

    public Command setFlywheelShooting() {
        return this.runOnce(() -> SuperStructure.Turret.FlywheelState = FlywheelState.SHOOTING);
    }

    public Command setFlywheelIdle() {
        return this.runOnce(() -> SuperStructure.Turret.FlywheelState = FlywheelState.IDLE);
    }

    public Command stopFlywheel() {
        return this.runOnce(() -> SuperStructure.Turret.FlywheelState = FlywheelState.STOPPED);
    }

    public Command setTargetingAuto() {
        return this.runOnce(() -> SuperStructure.Turret.TargetingState = TargetingState.AUTO);
    }

    public Command setTargetingManual() {
        return this.runOnce(() -> SuperStructure.Turret.TargetingState = TargetingState.MANUAL);
    }

    public Command stopTargeting() {
        return this.runOnce(() -> SuperStructure.Turret.TargetingState = TargetingState.STOPPED);
    }

}
