package frc.robot.subsystems.turret;

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

    public enum TargetingState {
        IDLE,
        SHOOT_MANUAL,
        SHOOT_AUTO,
        PASSING,
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

    private void actOnState(TurretInputsAutoLogged inputs) {
        // TODO: explain
        switch (inputs.TargetingState) {
            case SHOOT_MANUAL:
                _turret.controlFlywheel(_flywheelControl.withVelocity(_manualFlywheelVelocityRPS));
                _turret.controlYaw(_yawManualControl.withOutput(_manualYawSpeed));
                break;
            case SHOOT_AUTO:
            case IDLE:
                var target = FieldTargets.GetPassingPosition(SuperStructure.Swerve.EstimatedRobotPose);
                if (target == null) {
                    // We are not in the neutral zone, target the hub.
                    target = FieldTargets.GetHubPosition();
                }
                MutVector aimVector = calculateFinalTurretAimVector(target);

                var yaw = aimVector.getYaw();
                yaw += _manualYawSpeed * TurretMap.AUTO_AIM_YAW_TRIM_DEGREES;
                _turret.controlYaw(_yawControl.withPosition(yaw));

                // TODO: Implement pitch control once CAD finalizes turret
                var pitch = aimVector.getPitch();
                // <hood pitch implementation>

                // control flywheel
                if (inputs.TargetingState == TargetingState.IDLE) {
                    // Low speed for flywheel
                    _turret.controlFlywheel(_flywheelControl.withVelocity(TurretMap.FLYWHEEL_IDLE_VELOCITY_RPS));
                } else {
                    // Temp relation between flywheel speed and fuel velocity, will be replaced
                    // with a more concrete relation after testing
                    var targetVelocity = aimVector.getMagnitude();
                    var targetFlywheelOmega = (targetVelocity * (7 / 2)) / TurretMap.FLYWHEEL_RADIUS;
                    _turret.controlFlywheel(_flywheelControl.withVelocity(targetFlywheelOmega));
                }
                break;
            case STOPPED:
            default:
                _turret.controlFlywheel(_flywheelControl.withVelocity(0));
                _turret.controlYaw(_yawManualControl.withOutput(0));
                break;
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
        Logger.processInputs(getName(), SuperStructure.Turret);

        actOnState(SuperStructure.Turret);
    }

    public Command stopTurret() {
        return this.runOnce(() -> SuperStructure.Turret.TargetingState = TargetingState.IDLE);
    }
}
