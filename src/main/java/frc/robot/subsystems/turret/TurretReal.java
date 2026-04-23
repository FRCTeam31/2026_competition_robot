package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.CTREConverter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Container;
import frc.robot.Robot;

public class TurretReal implements ITurret {

    private SparkFlex _sparkFeed;
    private Servo _hoodServo;
    private SparkFlex _flywheelLeft;
    private SparkFlex _flywheelRight;
    private TalonSRX _turretRotator;
    private DigitalInput _turretResetLimitSwitch;
    private SparkClosedLoopController _flywheelClosedLoopController;
    private SparkClosedLoopController _hoodClosedLoopController;
    private PIDController _yawPidController;

    // Stored setpoints for on-target calculations
    private double _targetFlywheelVelocityRPM = 0;
    private double _targetHoodDegrees = 0;

    public TurretReal() {
        configureFlywheelMotors(TurretMap.FLYWHEEL_PID);
        configureSparkFeedMotor();
        configureTurretRotationMotor(TurretMap.TURRET_ROTATOR_PID);
        configureHoodServo();

        _turretResetLimitSwitch = new DigitalInput(TurretMap.TURRET_RESET_SWITCH_CHANNEL);
    }

    private void configureFlywheelMotors(ExtendedPIDConstants pid) {
        _flywheelLeft = new SparkFlex(TurretMap.FLYWHEEL_LEFT_CANID, MotorType.kBrushless);
        _flywheelRight = new SparkFlex(TurretMap.FLYWHEEL_RIGHT_CANID, MotorType.kBrushless);

        SparkFlexConfig leftConfig = new SparkFlexConfig();
        leftConfig.idleMode(IdleMode.kCoast);
        leftConfig.smartCurrentLimit(60, 40);
        leftConfig.encoder.positionConversionFactor(TurretMap.FLYWHEEL_GEAR_RATIO);
        leftConfig.encoder.velocityConversionFactor(TurretMap.FLYWHEEL_GEAR_RATIO);
        leftConfig.inverted(TurretMap.FLYWHEEL_LEFT_INVERTED);
        leftConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        leftConfig.closedLoop.feedForward.sva(pid.kS, pid.kV, pid.kA);
        leftConfig.closedLoopRampRate(TurretMap.FLYWHEEL_RAMP_PERIOD);
        leftConfig.closedLoop.positionWrappingEnabled(true);

        _flywheelLeft.clearFaults();
        _flywheelLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig rightConfig = new SparkFlexConfig();
        rightConfig.smartCurrentLimit(60, 40);
        rightConfig.follow(TurretMap.FLYWHEEL_LEFT_CANID, true);

        _flywheelRight.clearFaults();
        _flywheelRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _flywheelClosedLoopController = _flywheelLeft.getClosedLoopController();
    }

    private void configureSparkFeedMotor() {
        _sparkFeed = new SparkFlex(TurretMap.FEEDER_CANID, MotorType.kBrushless);
        var sparkConfig = new SparkFlexConfig()
                .inverted(TurretMap.FEEDER_INVERTED);
        sparkConfig.encoder.velocityConversionFactor(TurretMap.FEEDER_VELOCITY_CONVERSION_FACTOR);

        _sparkFeed.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configureTurretRotationMotor(ExtendedPIDConstants pid) {
        _turretRotator = new TalonSRX(TurretMap.TURRET_ROTATOR_CANID);
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        _turretRotator.setInverted(TurretMap.TURRET_ROTATOR_INVERTED);
        _turretRotator.setNeutralMode(NeutralMode.Brake);

        config.peakCurrentLimit = 60;
        config.continuousCurrentLimit = 50;
        config.peakCurrentDuration = 50;
        _turretRotator.enableCurrentLimit(true);

        // Configure PID constants in slot 0
        SlotConfiguration slot0 = new SlotConfiguration();
        slot0.kP = 0.2; // static value for testing
        // slot0.kI = 0;
        // slot0.kD = 0;
        slot0.kF = 0.1; // static value for testing
        config.slot0 = slot0;

        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        config.forwardSoftLimitEnable = true;
        config.forwardSoftLimitThreshold = 37000;
        config.reverseSoftLimitEnable = true;
        config.reverseSoftLimitThreshold = 6700;

        // Motion Magic configuration for smooth position control
        config.motionCruiseVelocity = TurretMap.YAW_MOTION_MAGIC_CRUISE_VELOCITY;
        config.motionAcceleration = TurretMap.YAW_MOTION_MAGIC_ACCELERATION;
        config.motionCurveStrength = 1;

        _turretRotator.configAllSettings(config);
        _turretRotator.clearStickyFaults();

        _yawPidController = pid.createPIDController(Robot.defaultPeriodSecs);
        _yawPidController.enableContinuousInput(0, 4096 * TurretMap.TURRET_GEAR_RATIO);
        Container.Dashboard.putData("Turret/YawPIDController", _yawPidController);
    }

    private void configureHoodServo() {
        _hoodServo = new Servo(TurretMap.HOOD_SERVO_CHANNEL);
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.TurretRotation = getTurretRotation();
        inputs.TurretRotationDegrees = inputs.TurretRotation.getDegrees();
        inputs.TurretRotationResetSwitch = !_turretResetLimitSwitch.get();
        // inputs.FlywheelVelocity = getFlywheelVelocity();
        inputs.FlywheelVelocityRPM = getFlywheelVelocity().in(RPM);
        inputs.FlywheelVoltage = _flywheelLeft.getAppliedOutput() * _flywheelLeft.getBusVoltage();
        inputs.YawVoltage = _turretRotator.getMotorOutputVoltage();
        inputs.HoodAngle = Degrees.of(_hoodServo.getAngle());
        inputs.FlywheelAngle = Rotations.of(_flywheelLeft.getEncoder().getPosition());

        // Compute on-target flags
        double flywheelToleranceRPM = _targetFlywheelVelocityRPM
                * TurretMap.FLYWHEEL_AT_SPEED_TOLERANCE_PERCENT / 100.0;
        inputs.FlywheelAtTargetSpeed = Math.abs(inputs.FlywheelVelocityRPM
                - _targetFlywheelVelocityRPM) <= flywheelToleranceRPM;
        inputs.YawOnTarget = _yawPidController.atSetpoint();
        inputs.HoodOnTarget = Math.abs(inputs.HoodAngle.in(Degrees)
                - _targetHoodDegrees) <= TurretMap.HOOD_ON_TARGET_TOLERANCE_DEGREES;
    }

    private Rotation2d getTurretRotation() {
        var magEncoderPosition = _turretRotator.getSelectedSensorPosition();

        return CTREConverter.CANcoderToRotation(magEncoderPosition, TurretMap.TURRET_GEAR_RATIO);
    }

    private MutAngularVelocity getFlywheelVelocity() {
        var leftMotorVelocity = _flywheelLeft.getEncoder().getVelocity();
        return RPM.mutable(leftMotorVelocity);
    }

    @Override
    public void controlFlywheel(AngularVelocity velocity) {
        _targetFlywheelVelocityRPM = velocity.in(RPM);
        _flywheelClosedLoopController.setSetpoint(velocity.magnitude(), ControlType.kVelocity);
    }

    @Override
    public void controlYawAngle(Angle angle) {
        var offsetAngle = angle.plus(Degrees.of(0));
        var pidOutput = _yawPidController.calculate(getTurretRotation().getRotations(), offsetAngle.in(Rotations));
        pidOutput = -MathUtil.clamp(pidOutput, -1, 1);
        Logger.recordOutput("Turret/YawPIDOutput", pidOutput);

        _turretRotator.set(TalonSRXControlMode.PercentOutput, pidOutput);
    }

    @Override
    public void setYawPercentOut(double percentOut) {
        _turretRotator.set(TalonSRXControlMode.PercentOutput, percentOut);
    }

    @Override
    public void controlHood(Angle angle) {
        if (Objects.equals(TurretMap.HOOD_ANGLE_RANGE_DEGREES, 0.0)) {
            return;
        }

        _hoodServo.set((TurretMap.HOOD_MAX_ANGLE_DEGREES - angle.in(Degrees)) / (TurretMap.HOOD_ANGLE_RANGE_DEGREES));
    }

    @Override
    public void setFeederSpeed(double speed) {
        _sparkFeed.set(speed);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        _flywheelLeft.setVoltage(volts);
        System.out.println("Setting flywheel voltage to: " + volts);
    }

    @Override
    public void setYawVoltage(double volts) {
        double busVoltage = _turretRotator.getBusVoltage();
        double percentOutput = volts / busVoltage;

        _turretRotator.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setHoodPercentOut(double percentOut) {
        _hoodServo.setSpeed(percentOut);
    }

    @Override
    public void setYawSensorPosition(Angle position) {
        var turretRotations = position.in(Degrees) / 360;
        var pinionRotations = turretRotations * TurretMap.TURRET_GEAR_RATIO;
        var encoderRotations = pinionRotations * 4096;
        _turretRotator.setSelectedSensorPosition(encoderRotations);
    }

    @Override
    public void setHoodSensorPosition(Angle position) {
        // Do nothing
    }
}
