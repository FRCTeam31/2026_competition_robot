package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;

public class TurretReal implements ITurret {

    private SparkFlex _sparkFeed;
    private SparkMax _sparkHood;
    private SparkFlex _flywheelLeft;
    private SparkFlex _flywheelRight;
    private TalonSRX _turretRotator;
    private DigitalInput _turretResetLimitSwitch;
    private SparkClosedLoopController _flywheelClosedLoopController;
    private SparkClosedLoopController _hoodClosedLoopController;

    // Stored setpoints for on-target calculations
    private double _targetFlywheelVelocityRPS = 0;
    private double _targetYawDegrees = 0;
    private double _targetHoodDegrees = 0;

    public TurretReal() {
        configureFlywheelMotors(TurretMap.FLYWHEEL_PID);
        configureSparkFeedMotor();
        configureTurretRotationMotor(TurretMap.TURRET_ROTATOR_PID);
        configureHoodMotor(TurretMap.HOOD_PID);

        _turretResetLimitSwitch = new DigitalInput(2); // Placeholder
    }

    private void configureFlywheelMotors(ExtendedPIDConstants pid) {
        _flywheelLeft = new SparkFlex(TurretMap.FLYWHEEL_LEFT_CANID, MotorType.kBrushless);
        _flywheelRight = new SparkFlex(TurretMap.FLYWHEEL_RIGHT_CANID, MotorType.kBrushless);

        SparkFlexConfig defaultConfig = new SparkFlexConfig();
        defaultConfig.idleMode(IdleMode.kCoast);
        defaultConfig.smartCurrentLimit(60, 40);
        defaultConfig.encoder.positionConversionFactor(1 / TurretMap.FLYWHEEL_GEAR_RATIO);
        defaultConfig.encoder.velocityConversionFactor(1 / TurretMap.FLYWHEEL_GEAR_RATIO);

        SparkFlexConfig leftConfig = defaultConfig;
        leftConfig.inverted(TurretMap.FLYWHEEL_LEFT_INVERTED);
        leftConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        leftConfig.closedLoop.feedForward.sva(pid.kS, pid.kV, pid.kA);
        leftConfig.closedLoopRampRate(TurretMap.FLYWHEEL_RAMP_PERIOD);

        _flywheelLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig rightConfig = defaultConfig;
        rightConfig.follow(TurretMap.FLYWHEEL_LEFT_CANID, true);

        _flywheelRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println(_flywheelRight.isFollower() + " : right is follower");
        System.out.println(_flywheelLeft.isFollower() + " : left is follower");

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

        config.peakCurrentLimit = 40;
        config.continuousCurrentLimit = 40;
        config.peakCurrentDuration = 0;
        _turretRotator.enableCurrentLimit(true);

        SlotConfiguration slot0 = new SlotConfiguration();
        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kF = pid.kV;
        config.slot0 = slot0;

        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        // TODO: set forward and reverse soft limits based on physical limits of the turret

        // Motion Magic configuration for smooth position control
        config.motionCruiseVelocity = TurretMap.YAW_MOTION_MAGIC_CRUISE_VELOCITY;
        config.motionAcceleration = TurretMap.YAW_MOTION_MAGIC_ACCELERATION;

        _turretRotator.configAllSettings(config);
        _turretRotator.clearStickyFaults();
    }

    private void configureHoodMotor(ExtendedPIDConstants pid) {
        _sparkHood = new SparkMax(TurretMap.HOOD_CAN_ID, MotorType.kBrushless);
        var sparkConfig = new SparkMaxConfig();

        sparkConfig.inverted(TurretMap.HOOD_INVERTED);
        sparkConfig.idleMode(IdleMode.kCoast);

        // TODO: Check current limits, ensure it can run but doesn't have enough power to destory the hood
        sparkConfig.smartCurrentLimit(25, 10);

        sparkConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        sparkConfig.closedLoop.feedForward.sva(pid.kS, pid.kV, pid.kA);

        // sparkConfig.encoder.inverted(TurretMap.HOOD_ENCODER_INVERTED);

        // MAXMotion configuration for smooth position control
        sparkConfig.closedLoop.maxMotion
                .cruiseVelocity(TurretMap.HOOD_MAX_MOTION_MAX_VELOCITY)
                .maxAcceleration(TurretMap.HOOD_MAX_MOTION_MAX_ACCELERATION)
                .allowedProfileError(TurretMap.HOOD_MAX_MOTION_ALLOWED_ERROR);

        // Set conversion factor
        sparkConfig.encoder.positionConversionFactor(1 / TurretMap.HOOD_GEAR_RATIO);

        _sparkHood.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _hoodClosedLoopController = _sparkHood.getClosedLoopController();
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.TurretRotation = getTurretRotation();
        inputs.TurretRotationResetSwitch = _turretResetLimitSwitch.get();
        inputs.FlywheelVelocity = getFlywheelVelocity();
        inputs.FlywheelVoltage = _flywheelLeft.getAppliedOutput() * _flywheelLeft.getBusVoltage();
        inputs.YawVoltage = _turretRotator.getMotorOutputVoltage();
        inputs.HoodAngle = Angle.ofBaseUnits(_sparkHood.getEncoder().getPosition(), Rotations);
        inputs.FlywheelAngle = Angle.ofBaseUnits(_flywheelLeft.getEncoder().getPosition(), Rotations);

        // Compute on-target flags
        double flywheelToleranceRPS = _targetFlywheelVelocityRPS
                * TurretMap.FLYWHEEL_AT_SPEED_TOLERANCE_PERCENT / 100.0;
        inputs.FlywheelAtTargetSpeed = Math.abs(inputs.FlywheelVelocity.in(RotationsPerSecond)
                - _targetFlywheelVelocityRPS) <= flywheelToleranceRPS;
        inputs.YawOnTarget = Math.abs(inputs.TurretRotation.getDegrees()
                - _targetYawDegrees) <= TurretMap.YAW_ON_TARGET_TOLERANCE_DEGREES;
        inputs.HoodOnTarget = Math.abs(inputs.HoodAngle.in(Degrees)
                - _targetHoodDegrees) <= TurretMap.HOOD_ON_TARGET_TOLERANCE_DEGREES;
    }

    private Rotation2d getTurretRotation() {
        var motorRotation = _turretRotator.getSelectedSensorPosition();
        // 4096 ticks per pinion revolution × TURRET_GEAR_RATIO pinion revs per turret revolution
        var turretRotation = motorRotation / (4096.0 * TurretMap.TURRET_GEAR_RATIO);

        return Rotation2d.fromRotations(turretRotation);
    }

    private MutAngularVelocity getFlywheelVelocity() {
        var leftMotorVelocity = _flywheelLeft.getEncoder().getVelocity();
        var rightMotorVelocity = _flywheelRight.getEncoder().getVelocity();

        var averageVelocity = (leftMotorVelocity + rightMotorVelocity) / 2;

        return RPM.mutable(averageVelocity);
    }

    @Override
    public void controlFlywheel(AngularVelocity velocity) {
        _targetFlywheelVelocityRPS = velocity.in(RotationsPerSecond);
        _flywheelClosedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void controlYawAngle(Angle angle) {
        _targetYawDegrees = angle.in(Degrees);
        _turretRotator.set(TalonSRXControlMode.MotionMagic,
                CTREConverter.degreesToCANcoder(_targetYawDegrees, TurretMap.TURRET_GEAR_RATIO));
    }

    @Override
    public void setYawPercentOut(double percentOut) {
        _turretRotator.set(TalonSRXControlMode.PercentOutput, percentOut);
    }

    @Override
    public void controlHood(Angle angle) {
        _targetHoodDegrees = angle.in(Degrees);
        _hoodClosedLoopController.setSetpoint(angle.in(Rotations), ControlType.kMAXMotionPositionControl);
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
        _sparkHood.set(percentOut);
    }

    @Override
    public void setYawSensorPosition(Angle position) {
        _turretRotator.setSelectedSensorPosition(
                CTREConverter.degreesToCANcoder(position.in(Degrees), TurretMap.TURRET_GEAR_RATIO));
    }

    @Override
    public void setHoodSensorPosition(Angle position) {
        _sparkHood.getEncoder().setPosition(position.in(Rotations));
    }
}
