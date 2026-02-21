package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.reflect.Array;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    private boolean _isManualControl = false;

    // DC Motor Simulation
    private DCMotorSim _hoodMotor;

    // Hood and feeder state
    private double _hoodSpeed = 0;
    private double _feederSpeed = 0;

    // Simulated turret motion constants
    private static final double TURRET_CRUISE_VELOCITY_RPS = 100.0 / TurretMap.TURRET_GEAR_RATIO;
    private static final double TURRET_ACCELERATION_RPS2 = 200.0 / TurretMap.TURRET_GEAR_RATIO;

    public TurretSim() {
        _hoodMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNeo550(1),
                        4e-6,
                        1),
                DCMotor.getNeo550(1),
                0, 0);
    }

    /**
     * Call this each cycle from simulationPeriodic to advance turret position.
     * @param timestepSeconds the simulation timestep (typically 0.02s)
     */
    public void updateSimulation(double timestepSeconds) {
        if (_isManualControl) {
            // In manual (DutyCycleOut) mode, apply manual speed directly as velocity
            _turretVelocityRPS = _turretManualSpeed * TURRET_CRUISE_VELOCITY_RPS;
            _turretPositionRotations += _turretVelocityRPS * timestepSeconds;
            // Keep target in sync so switching to position mode doesn't cause a jump
            _turretTargetPositionRotations = _turretPositionRotations;
            return;
        }

        // Position (MotionMagic) mode: trapezoidal motion toward the target position
        double error = _turretTargetPositionRotations - _turretPositionRotations;

        // Wrap error to [-0.5, 0.5] rotations for continuous wrap behavior
        error = error - Math.round(error);

        if (Math.abs(error) < 0.01 && Math.abs(_turretVelocityRPS) < 0.015) {
            _turretPositionRotations = _turretTargetPositionRotations;
            _turretVelocityRPS = 0;
        } else {
            // Calculate stopping distance (signed — how far we'd travel while decelerating to zero)
            double stoppingDistance = (_turretVelocityRPS * Math.abs(_turretVelocityRPS)) /
                    (2.0 * TURRET_ACCELERATION_RPS2);

            // If we're overshooting (velocity carries us past the target), decelerate
            // Otherwise, accelerate toward the target
            boolean shouldDecelerate = Math.abs(stoppingDistance) >= Math.abs(error)
                    && Math.signum(stoppingDistance) == Math.signum(error);

            if (shouldDecelerate) {
                // Decelerate: reduce velocity magnitude toward zero
                double decel = Math.signum(_turretVelocityRPS) * TURRET_ACCELERATION_RPS2 * timestepSeconds;
                if (Math.abs(decel) > Math.abs(_turretVelocityRPS)) {
                    _turretVelocityRPS = 0;
                } else {
                    _turretVelocityRPS -= decel;
                }
            } else {
                // Accelerate toward target (in the direction of the error)
                _turretVelocityRPS += Math.signum(error) * TURRET_ACCELERATION_RPS2 * timestepSeconds;
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
        inputs.HoodAngle = _hoodMotor.getAngularPosition();
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
            _isManualControl = false;
        } else if (request instanceof DutyCycleOut dutyCycleRequest) {
            _turretManualSpeed = dutyCycleRequest.Output;
            _isManualControl = true;
        }
    }

    @Override
    public void controlHood(double percentOut) {
        _hoodMotor.setAngularVelocity(percentOut * TurretMap.HOOD_SIM_MAX_SPEED.magnitude());
    }

    @Override
    public void setFeederSpeed(double speed) {
        _feederSpeed = speed;
    }
}