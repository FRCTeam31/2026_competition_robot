package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.prime.control.ExtendedPIDConstants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
    // TODO: Convert to TalonSRX
    private TalonFX _turretRotator;
    private DigitalInput _turretResetLimitSwitch;

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

        SparkFlexConfig leftConfig = new SparkFlexConfig();

        leftConfig.inverted(TurretMap.FLYWHEEL_LEFT_INVERTED);
        leftConfig.idleMode(IdleMode.kCoast);

        leftConfig.smartCurrentLimit(60, 40);

        leftConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        leftConfig.closedLoopRampRate(TurretMap.FLYWHEEL_RAMP_PERIOD);

        // Copy the left config but modify it to follow the left motor
        SparkFlexConfig rightConfig = leftConfig;
        rightConfig.follow(TurretMap.FLYWHEEL_LEFT_CANID, true);

        // Apply configuration
        _flywheelLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _flywheelRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureSparkFeedMotor() {
        _sparkFeed = new SparkFlex(TurretMap.FEEDER_CANID, MotorType.kBrushless);
        var sparkConfig = new SparkFlexConfig()
                .inverted(TurretMap.FEEDER_INVERTED);
        sparkConfig.encoder.velocityConversionFactor(TurretMap.FEEDER_VELOCITY_CONVERSION_FACTOR);

        _sparkFeed.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configureTurretRotationMotor(ExtendedPIDConstants pid) {
        _turretRotator = new TalonFX(TurretMap.TURRET_ROTATOR_CANID);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = TurretMap.TURRET_ROTATOR_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kS = pid.kS;
        slot0.kV = pid.kV;
        slot0.kA = pid.kA;
        config.Slot0 = slot0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretMap.TURRET_GEAR_RATIO;

        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicCruiseVelocity = 100; // TODO: Tune this
        motionMagic.MotionMagicAcceleration = 200; // TODO: Tune this
        motionMagic.MotionMagicJerk = 1600; // TODO: Tune this
        config.MotionMagic = motionMagic;

        config.ClosedLoopGeneral.ContinuousWrap = !TurretMap.YAW_DEADZONE_ENABLED;

        _turretRotator.getConfigurator().apply(config);
        _turretRotator.clearStickyFaults();
    }

    private void configureHoodMotor(ExtendedPIDConstants pid) {
        _sparkHood = new SparkMax(TurretMap.HOOD_CAN_ID, MotorType.kBrushless);
        var sparkConfig = new SparkMaxConfig();

        sparkConfig.inverted(TurretMap.HOOD_INVERTED);
        sparkConfig.idleMode(IdleMode.kCoast);

        sparkConfig.smartCurrentLimit(40, 30);

        sparkConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);

        _sparkHood.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.TurretRotation = getTurretRotation();
        inputs.TurretRotationResetSwitch = _turretResetLimitSwitch.get();
        inputs.FlywheelVelocity = getFlywheelVelocity();
        inputs.FlywheelVoltage = _flywheelLeft.getBusVoltage();
        inputs.YawVoltage = _turretRotator.getMotorVoltage().getValueAsDouble();
        inputs.HoodAngle = Angle.ofBaseUnits(_sparkHood.getEncoder().getPosition(), Rotations);
    }

    private Rotation2d getTurretRotation() {
        var motorRotation = _turretRotator.getPosition().getValueAsDouble();
        var turretRotation = motorRotation / TurretMap.TURRET_GEAR_RATIO;

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
        _flywheelLeft.getClosedLoopController().setSetpoint(velocity.in(RotationsPerSecond), ControlType.kVelocity);
    }

    @Override
    public void controlYaw(ControlRequest yawRequest) {
        _turretRotator.setControl(yawRequest);
    }

    @Override
    public void controlHood(Angle angle) {
        _sparkHood.getClosedLoopController().setSetpoint(angle.in(Degrees), ControlType.kPosition);
    }

    @Override
    public void setFeederSpeed(double speed) {
        _sparkFeed.set(speed);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        _flywheelLeft.setVoltage(volts);
    }

    @Override
    public void setYawVoltage(double volts) {
        _turretRotator.setVoltage(volts);
    }

    @Override
    public void setHoodPercentOut(double percentOut) {
        _sparkHood.set(percentOut);
    }
}
