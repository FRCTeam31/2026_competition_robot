package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;

/**
 * Real hardware implementation of the Climb subsystem IO.
 * Controls a SparkFlex motor with MAXMotion position control,
 * two double solenoids (friction brake and support), and a digital limit switch.
 */
public class ClimbReal implements IClimb {

    // Devices
    private SparkFlex _climbMotor;
    private SparkClosedLoopController _closedLoopController;
    private DoubleSolenoid _frictionBrakeSolenoid;
    private DoubleSolenoid _supportSolenoid;
    private DigitalInput _limitSwitch;

    public ClimbReal() {
        configureClimbMotor();

        _frictionBrakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.FRICTION_BRAKE_FORWARD_CHANNEL,
                ClimbMap.FRICTION_BRAKE_REVERSE_CHANNEL);

        _supportSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.SUPPORT_FORWARD_CHANNEL,
                ClimbMap.SUPPORT_REVERSE_CHANNEL);

        _limitSwitch = new DigitalInput(ClimbMap.LIMIT_SWITCH_CHANNEL);
    }

    /** Configures the SparkFlex climb motor with MAXMotion position control, soft limits, and current limits. */
    private void configureClimbMotor() {
        _climbMotor = new SparkFlex(ClimbMap.CLIMB_MOTOR_CANID, MotorType.kBrushless);
        var config = new SparkFlexConfig();
        var pid = ClimbMap.CLIMB_PID;

        config.inverted(ClimbMap.CLIMB_MOTOR_INVERTED);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(70, 60);

        // PID + feedforward for position control
        config.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        config.closedLoop.feedForward.sva(pid.kS, pid.kV, pid.kA);

        // MAXMotion configuration for smooth position control
        config.closedLoop.maxMotion
                .cruiseVelocity(ClimbMap.MAX_MOTION_MAX_VELOCITY)
                .maxAcceleration(ClimbMap.MAX_MOTION_MAX_ACCELERATION)
                .allowedProfileError(ClimbMap.MAX_MOTION_ALLOWED_ERROR);

        // Soft limits to prevent the motor from over-extending or over-retracting
        config.softLimit
                .forwardSoftLimit(ClimbMap.FORWARD_SOFT_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ClimbMap.REVERSE_SOFT_LIMIT)
                .reverseSoftLimitEnabled(true);

        _climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _climbMotor.clearFaults();

        _closedLoopController = _climbMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ClimbInputsAutoLogged inputs) {
        inputs.LowerLimitSwitch = _limitSwitch.get();
        inputs.MotorRotations = _climbMotor.getEncoder().getPosition();
    }

    @Override
    public void setClimbPosition(double rotations) {
        _closedLoopController.setSetpoint(rotations, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopClimb() {
        _climbMotor.stopMotor();
    }

    @Override
    public void controlSupport(DoubleSolenoid.Value value) {
        _supportSolenoid.set(value);
    }

    @Override
    public void controlFrictionBrake(DoubleSolenoid.Value value) {
        _frictionBrakeSolenoid.set(value);
    }

    @Override
    public void zeroEncoder() {
        _climbMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setClimbPercentOut(double percentOut) {
        _climbMotor.set(percentOut);
    }
}
