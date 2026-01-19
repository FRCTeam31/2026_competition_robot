package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.prime.control.ExtendedPIDConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.turret.Turret.FlywheelStates;
import frc.robot.subsystems.turret.Turret.TargetingStates;

public class TurretReal implements ITurret {

    private SparkFlex _sparkFeed;
    private TalonFX _flywheelLeft;
    private TalonFX _flywheelRight;
    private TalonFX _turretRotator;
    private DigitalInput _turretResetLimitSwitch;

    public TurretReal() {
        configureFlywheelMotors(TurretMap.FLYWHEEL_PID);
        configureSparkFeedMotor();
        configureTurretRotationMotor(TurretMap.TURRET_ROTATOR_PID);
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

        config.ClosedLoopGeneral.ContinuousWrap = true; // TODO: Determine if needed

        _turretRotator.getConfigurator().apply(config);
        _turretRotator.clearStickyFaults();
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.TurretRotation = getTurretRotation();
        inputs.TurretRotationResetSwitch = _turretResetLimitSwitch.get();
        inputs.FlywheelVelocity = getFlywheelVelocity();

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

    // CTRE Control Requests
    private final VelocityVoltage _flywheelControl = new VelocityVoltage(0);
    private final MotionMagicVoltage _rotatorControl = new MotionMagicVoltage(0);
    private final DutyCycleOut _rotatorManualControl = new DutyCycleOut(0);

    @Override
    public void controlFlywheel(FlywheelStates state, double manualTargetVelocityRPS) {
        switch (state) {
            case IDLE:
                _flywheelLeft.setControl(_flywheelControl.withVelocity(TurretMap.FLYWHEEL_IDLE_VELOCITY_RPS));
                break;
            case SHOOT_MANUAL:
                _flywheelLeft.setControl(_flywheelControl.withVelocity(manualTargetVelocityRPS));
                break;
            case SHOOT_AUTO:
                // TODO: Calculate target velocity based on distance to target
                break;
            case STOPPED:
            default:
                _flywheelLeft.stopMotor();
                break;
        }
    }

    @Override
    public void controlTargeting(TargetingStates state, double manualControlSpeed) {
        switch (state) {
            case MANUAL_CONTROL:
                _turretRotator.setControl(_rotatorManualControl.withOutput(manualControlSpeed));
                break;
            case AUTO_ASSISTED:
            default:
                // TODO: Implement auto-assisted targeting
                // _turretRotator.setControl(_rotatorControl.withPosition(robotRelativeTargetAngle));
                break;
        }
    }

    @Override
    public void setFeederSpeed(double speed) {
        _sparkFeed.set(speed);
    }

}
