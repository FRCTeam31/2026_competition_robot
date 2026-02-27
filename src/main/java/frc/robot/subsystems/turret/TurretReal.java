package frc.robot.subsystems.turret;

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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;

public class TurretReal implements ITurret {

    private SparkFlex _sparkFeed;
    private SparkFlex _sparkHood;
    private SparkMax _flywheelLeft;
    private SparkMax _flywheelRight;
    private SparkClosedLoopController _flywheelClosedLoopController;
    private TalonFX _turretRotator;
    private DigitalInput _turretResetLimitSwitch;

    public TurretReal() {
        configureFlywheelMotors(TurretMap.FLYWHEEL_PID);
        configureSparkFeedMotor();
        configureTurretRotationMotor(TurretMap.TURRET_ROTATOR_PID);
        configureHoodMotor();
    }

    private void configureFlywheelMotors(ExtendedPIDConstants pid) {
        _flywheelLeft = new SparkMax(TurretMap.FLYWHEEL_LEFT_CANID, MotorType.kBrushless);
        _flywheelRight = new SparkMax(TurretMap.FLYWHEEL_RIGHT_CANID, MotorType.kBrushless);

        // Configure left (leader) motor
        var leftConfig = new SparkMaxConfig();
        leftConfig.inverted(TurretMap.FLYWHEEL_LEFT_INVERTED);
        leftConfig.idleMode(IdleMode.kCoast);
        leftConfig.smartCurrentLimit(60);
        leftConfig.closedLoopRampRate(TurretMap.FLYWHEEL_RAMP_PERIOD);

        // SparkMax closed-loop supports P, I, D, and velocityFF.
        // kS (static friction) and kA (acceleration) from ExtendedPIDConstants are not
        // directly supported by SparkMax onboard control and are intentionally omitted.
        leftConfig.closedLoop
                .p(pid.kP)
                .i(pid.kI)
                .d(pid.kD)
                .velocityFF(pid.kV);

        _flywheelLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure right (follower) motor - follows left with opposite direction
        var rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.smartCurrentLimit(60);
        rightConfig.follow(TurretMap.FLYWHEEL_LEFT_CANID, true);

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
        inputs.FlywheelVoltage = _flywheelLeft.getAppliedOutput() * _flywheelLeft.getBusVoltage();
        inputs.YawVoltage = _turretRotator.getMotorVoltage().getValueAsDouble();
        inputs.HoodAngle = Angle.ofBaseUnits(_sparkHood.getEncoder().getPosition(), Rotations);
    }

    private Rotation2d getTurretRotation() {
        var motorRotation = _turretRotator.getPosition().getValueAsDouble();
        var turretRotation = motorRotation / TurretMap.TURRET_GEAR_RATIO;

        return Rotation2d.fromRotations(turretRotation);
    }

    private MutAngularVelocity getFlywheelVelocity() {
        // SparkMax encoder returns RPM; convert to rotations per second
        var leftMotorVelocityRPM = _flywheelLeft.getEncoder().getVelocity();
        var rightMotorVelocityRPM = _flywheelRight.getEncoder().getVelocity();

        var averageVelocityRPS = (leftMotorVelocityRPM + rightMotorVelocityRPM) / 2.0 / 60.0;

        return RotationsPerSecond.mutable(averageVelocityRPS);
    }

    @Override
    public void controlFlywheel(double targetRotationsPerSecond) {
        // SparkMax closed-loop velocity control expects RPM
        double targetRPM = targetRotationsPerSecond * 60.0;
        _flywheelClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
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
        // Right motor follows left automatically
    }

    @Override
    public void setYawVoltage(double volts) {
        _turretRotator.setVoltage(volts);
    }

}
