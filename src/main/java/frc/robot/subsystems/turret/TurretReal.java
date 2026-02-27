package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.prime.control.ExtendedPIDConstants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;

public class TurretReal implements ITurret {

    private SparkFlex _sparkFeed;
    private SparkFlex _sparkHood;
    private TalonFX _flywheelLeft;
    private TalonFX _flywheelRight;
    private TalonFX _turretRotator;
    private DigitalInput _turretResetLimitSwitch;

    public TurretReal() {
        configureFlywheelMotors(TurretMap.FLYWHEEL_PID);
        configureSparkFeedMotor();
        configureTurretRotationMotor(TurretMap.TURRET_ROTATOR_PID);
        configureHoodMotor();

        _turretResetLimitSwitch = new DigitalInput(2); // Placeholder
    }

    private void configureFlywheelMotors(ExtendedPIDConstants pid) {
        _flywheelLeft = new TalonFX(TurretMap.FLYWHEEL_LEFT_CANID);
        _flywheelRight = new TalonFX(TurretMap.FLYWHEEL_RIGHT_CANID);
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.Inverted = TurretMap.FLYWHEEL_LEFT_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.CurrentLimits.StatorCurrentLimit = 80;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 60;

        leftConfig.Voltage.PeakForwardVoltage = 12;
        leftConfig.Voltage.PeakReverseVoltage = -12;

        leftConfig.Feedback.SensorToMechanismRatio = 1;
        leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kS = pid.kS;
        slot0.kV = pid.kV;
        slot0.kA = pid.kA;
        leftConfig.Slot0 = slot0;

        leftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = TurretMap.FLYWHEEL_RAMP_PERIOD;

        leftConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        leftConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

        _flywheelLeft.getConfigurator().apply(leftConfig);
        _flywheelLeft.clearStickyFaults();

        // Right is inverted follower
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        _flywheelRight.getConfigurator().apply(rightConfig);

        _flywheelRight.setControl(new Follower(_flywheelLeft.getDeviceID(), MotorAlignmentValue.Opposed));
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

    private void configureHoodMotor() {
        _sparkHood = new SparkFlex(TurretMap.HOOD_CAN_ID, MotorType.kBrushless);
        var sparkConfig = new SparkFlexConfig()
                .inverted(TurretMap.HOOD_INVERTED);

        _sparkHood.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.TurretRotation = getTurretRotation();
        inputs.TurretRotationResetSwitch = _turretResetLimitSwitch.get();
        inputs.FlywheelVelocity = getFlywheelVelocity();
        inputs.FlywheelVoltage = _flywheelLeft.getMotorVoltage().getValueAsDouble();
        inputs.YawVoltage = _turretRotator.getMotorVoltage().getValueAsDouble();
        inputs.HoodAngle = Angle.ofBaseUnits(_sparkHood.getEncoder().getPosition(), Rotations);
    }

    private Rotation2d getTurretRotation() {
        var motorRotation = _turretRotator.getPosition().getValueAsDouble();
        var turretRotation = motorRotation / TurretMap.TURRET_GEAR_RATIO;

        return Rotation2d.fromRotations(turretRotation);
    }

    private MutAngularVelocity getFlywheelVelocity() {
        var leftMotorVelocity = _flywheelLeft.getVelocity().getValueAsDouble();
        var rightMotorVelocity = _flywheelRight.getVelocity().getValueAsDouble();

        var averageVelocity = (leftMotorVelocity + rightMotorVelocity) / 2;

        return RotationsPerSecond.mutable(averageVelocity);
    }

    @Override
    public void controlFlywheel(ControlRequest request) {
        _flywheelLeft.setControl(request);
    }

    @Override
    public void controlYaw(ControlRequest yawRequest) {
        _turretRotator.setControl(yawRequest);
    }

    @Override
    public void controlHood(double percentOut) {
        _sparkHood.set(percentOut);
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
}
