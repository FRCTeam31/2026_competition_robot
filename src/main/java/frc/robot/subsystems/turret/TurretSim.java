package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import org.prime.subsystems.turret.TurretDeadZoneHelper;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

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

    // SysId voltage tracking
    private double _flywheelVoltage = 0;
    private double _yawVoltage = 0;

    // Simulated turret motion constants
    private static final double TURRET_CRUISE_VELOCITY_RPS = 100.0 / TurretMap.TURRET_GEAR_RATIO;
    private static final double TURRET_ACCELERATION_RPS2 = 200.0 / TurretMap.TURRET_GEAR_RATIO;

    // Dead zone enforcement for realistic simulation
    private final TurretDeadZoneHelper _deadZoneHelper = new TurretDeadZoneHelper(
            TurretMap.DEADZONE_START_DEGREES, TurretMap.DEADZONE_END_DEGREES);

    public TurretSim() {
        _hoodMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getNeo550(1),
                        4e-6, // MOI from AI, will likely have little effect
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

            // Hard-stop at dead zone edges
            if (TurretMap.YAW_DEADZONE_ENABLED) {
                clampAtDeadZone();
            }
            return;
        }

        // Position (MotionMagic) mode: trapezoidal motion toward the target position
        double error = _turretTargetPositionRotations - _turretPositionRotations;

        // When the dead zone is enabled, do NOT wrap the error -- the turret is not
        // continuous. When disabled, wrap to [-0.5, 0.5] for continuous behaviour.
        if (!TurretMap.YAW_DEADZONE_ENABLED) {
            error = error - Math.round(error);
        }

        if (Math.abs(error) < 0.01 && Math.abs(_turretVelocityRPS) < 0.015) {
            _turretPositionRotations = _turretTargetPositionRotations;
            _turretVelocityRPS = 0;
        } else {
            // Calculate stopping distance (signed -- how far we'd travel while decelerating to zero)
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

        // Hard-stop at dead zone edges
        if (TurretMap.YAW_DEADZONE_ENABLED) {
            clampAtDeadZone();
        }
    }

    /**
     * Simulates the physical hard stop at dead zone edges. If the turret has
     * drifted into the dead zone, snap it back to the nearest edge and zero
     * velocity (as if it hit a wall).
     */
    private void clampAtDeadZone() {
        if (_deadZoneHelper.isInDeadZone(_turretPositionRotations)) {
            // Snap to whichever edge is closest
            _turretPositionRotations = _deadZoneHelper.computeLegalSetpoint(
                    _turretPositionRotations, _turretPositionRotations);
            _turretVelocityRPS = 0;
            _turretTargetPositionRotations = _turretPositionRotations;
        }
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        updateSimulation(Robot.defaultPeriodSecs);

        inputs.TurretRotation = Rotation2d.fromRotations(_turretPositionRotations);
        inputs.TurretRotationResetSwitch = (_turretPositionRotations <= 0.001);
        inputs.FlywheelVelocity = RotationsPerSecond.mutable(_flywheelVelocityRPS);
        inputs.FlywheelVoltage = _flywheelVoltage;
        inputs.YawVoltage = _yawVoltage;
        inputs.HoodAngle = Angle.ofBaseUnits(
                _hoodMotor.getAngularPosition().in(Radians) * TurretMap.HOOD_GEAR_RADIUS.in(Meters),
                Radians // Check units
        );
    }

    @Override
    public void controlFlywheel(double targetRotationsPerSecond) {
        _flywheelVelocityRPS = targetRotationsPerSecond;
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

    @Override
    public void setFlywheelVoltage(double volts) {
        _flywheelVoltage = volts;
        // Approximate velocity from voltage for simulation
        _flywheelVelocityRPS = volts * 8.0; // rough approximation
    }

    @Override
    public void setYawVoltage(double volts) {
        _yawVoltage = volts;
        _turretManualSpeed = volts / 12.0;
        _isManualControl = true;
    }
}