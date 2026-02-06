package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import java.util.stream.DoubleStream;

import org.littletonrobotics.junction.Logger;
import org.prime.util.MutVector;

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

    private DoubleSupplier _yawSupplier;
    private DoubleSupplier _pitchSupplier;

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

    public enum FeedState {
        FORWARDS,
        REVERSED,
        STOPPED
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

    public MutVector calculateFinalTurretAimVector(Pose3d targetPose) {
        calculateTargetVectorFromRobotPose(targetPose);

        if (TurretMap.AUTO_MOTION_COMPENSATION) {
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
    }

    public MutVector calculateTargetVectorFromRobotPose(Pose3d targetPose) {
        var robotPose = SuperStructure.Swerve.EstimatedRobotPose;
        var deltaX = robotPose.getX() - targetPose.getX();
        var deltaY = robotPose.getY() - targetPose.getY();

        var yaw = Math.atan(deltaY / deltaX);

        var distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        var hubHeight = targetPose.getZ();
        var maxHeight = hubHeight + TurretMap.HUB_OVERSHOOT_HEIGHT;
        var turretHeight = TurretMap.TURRET_HEIGHT_ABOVE_GROUND;

        // TODO: double-check these equations
        double pitch = Math.atan(
                (2 * (maxHeight - turretHeight) + Math.sqrt((maxHeight - turretHeight) * (maxHeight - turretHeight)))
                        / distance);
        double velocity = Math.sqrt(2 * 9.81 * (hubHeight - turretHeight)) / Math.sin(pitch);

        _mutNominalTargetVector.setPolar(velocity, pitch, yaw);
        return _mutNominalTargetVector;
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
            aimVector = calculateFinalTurretAimVector(target);
        }

        // TODO: explain
        switch (inputs.TargetingState) {
            // TODO: Implement pitch control once CAD finalizes turret
            case MANUAL:
                _turret.controlYaw(
                        _yawManualControl.withOutput(TurretMap.TURRET_MAX_SPEED * _yawSupplier.getAsDouble()));

                _turret.controlHood(_pitchSupplier.getAsDouble()); // <hood pitch implementation>
                break;
            case AUTO:
                var yaw = aimVector.getYaw();
                yaw += _manualYawSpeed * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;
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

    public Command setFeedForward() {
        return this.runOnce(() -> SuperStructure.Turret.FeedState = FeedState.FORWARDS);
    }

    public Command setFeedReverse() {
        return this.runOnce(() -> SuperStructure.Turret.FeedState = FeedState.REVERSED);
    }

    public Command stopFeed() {
        return this.runOnce(() -> SuperStructure.Turret.FeedState = FeedState.STOPPED);
    }

    // These won't go down here

}
