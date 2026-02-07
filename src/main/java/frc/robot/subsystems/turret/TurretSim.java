package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

@SuppressWarnings("unused")
public class TurretSim implements ITurret {

    // Flywheel state
    private double _flywheelVelocityRPS = 0;

    // Turret rotation state
    private double _turretPositionRotations = 0;
    private double _turretTargetPositionRotations = 0;
    private double _turretManualSpeed = 0;
    private double _turretVelocityRPS = 0;

    // Hood and feeder state
    private double _hoodSpeed = 0;
    private double _feederSpeed = 0;

    // Simulated turret motion constants
    private static final double TURRET_CRUISE_VELOCITY_RPS = 100.0 / TurretMap.TURRET_GEAR_RATIO;
    private static final double TURRET_ACCELERATION_RPS2 = 200.0 / TurretMap.TURRET_GEAR_RATIO;

    public TurretSim() {
    }

    /**
     * Call this each cycle from simulationPeriodic to advance turret position.
     * @param timestepSeconds the simulation timestep (typically 0.02s)
     */
    public void updateSimulation(double timestepSeconds) {
        // Simple trapezoidal-ish motion toward the target position
        double error = _turretTargetPositionRotations - _turretPositionRotations;

        // Wrap error to [-0.5, 0.5] rotations for continuous wrap behavior
        error = error - Math.round(error);

        if (Math.abs(error) < 0.001) {
            _turretPositionRotations = _turretTargetPositionRotations;
            _turretVelocityRPS = 0;
        } else {
            double sign = Math.signum(error);

            // Accelerate or decelerate toward target
            double stoppingDistance = (_turretVelocityRPS * _turretVelocityRPS) /
                    (2.0 * TURRET_ACCELERATION_RPS2);

            if (stoppingDistance >= Math.abs(error)) {
                // Decelerate
                _turretVelocityRPS -= sign * TURRET_ACCELERATION_RPS2 * timestepSeconds;
                if (Math.abs(_turretVelocityRPS) < TURRET_ACCELERATION_RPS2 * timestepSeconds) {
                    _turretVelocityRPS = 0;
                }
            } else {
                // Accelerate up to cruise velocity
                _turretVelocityRPS += sign * TURRET_ACCELERATION_RPS2 * timestepSeconds;
                _turretVelocityRPS = Math.max(
                        Math.min(_turretVelocityRPS, TURRET_CRUISE_VELOCITY_RPS),
                        -TURRET_CRUISE_VELOCITY_RPS);
            }

            _turretPositionRotations += _turretVelocityRPS * timestepSeconds;
        }
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        updateSimulation(Robot.defaultPeriodSecs);

        inputs.TurretRotation = Rotation2d.fromRotations(_turretPositionRotations);
        inputs.TurretRotationResetSwitch = (_turretPositionRotations <= 0.001);
        inputs.FlywheelVelocity = RotationsPerSecond.mutable(_flywheelVelocityRPS);
    }

    @Override
    public void controlFlywheel(ControlRequest request) {
        // Extract the target velocity from known velocity control request types.
        // For unrecognized types, the flywheel velocity is left unchanged.
        if (request instanceof VelocityVoltage velocityRequest) {
            _flywheelVelocityRPS = velocityRequest.Velocity;
        }
    }

    @Override
    public void controlYaw(ControlRequest request) {
        // Extract the target position from known position control request types.
        if (request instanceof MotionMagicVoltage motionMagicRequest) {
            _turretTargetPositionRotations = motionMagicRequest.Position;
        } else if (request instanceof DutyCycleOut dutyCycleRequest) {
            _turretManualSpeed = dutyCycleRequest.Output;
        }
    }

    @Override
    public void controlHood(double speed) {
        _hoodSpeed = speed;
    }

    @Override
    public void setFeederSpeed(double speed) {
        _feederSpeed = speed;
    }
}