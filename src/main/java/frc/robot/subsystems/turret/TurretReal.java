package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.prime.control.ExtendedPIDConstants;

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

        SparkFlexConfig rightConfig = defaultConfig;
        rightConfig.follow(TurretMap.FLYWHEEL_LEFT_CANID, true);

        SparkFlexConfig leftConfig = defaultConfig;
        leftConfig.inverted(TurretMap.FLYWHEEL_LEFT_INVERTED);
        leftConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        leftConfig.closedLoopRampRate(TurretMap.FLYWHEEL_RAMP_PERIOD);

        _flywheelLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

        // TODO: Determine if needed, if so convert to v5
        // MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        // motionMagic.MotionMagicCruiseVelocity = 100; // TODO: Tune this
        // motionMagic.MotionMagicAcceleration = 200; // TODO: Tune this
        // motionMagic.MotionMagicJerk = 1600; // TODO: Tune this
        // config.MotionMagic = motionMagic;

        _turretRotator.configAllSettings(config);
        _turretRotator.clearStickyFaults();
    }

    private void configureHoodMotor(ExtendedPIDConstants pid) {
        _sparkHood = new SparkMax(TurretMap.HOOD_CAN_ID, MotorType.kBrushless);
        var sparkConfig = new SparkMaxConfig();

        sparkConfig.inverted(TurretMap.HOOD_INVERTED);
        sparkConfig.idleMode(IdleMode.kCoast);

        sparkConfig.smartCurrentLimit(40, 30);

        sparkConfig.closedLoop.pid(pid.kP, pid.kI, pid.kD);
        sparkConfig.closedLoop.feedForward.sva(pid.kS, pid.kV, pid.kA);

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
    }

    private int getRotatorEncoderTicksFromTurretYaw(Angle angle) {
        var degrees = angle.in(Degrees);
        return TurretMap.TURRET_YAW_ENCODER_TICKS_PER_TURRET_DEGREE * (int) degrees;
    }

    private Rotation2d getTurretRotation() {
        var motorRotation = _turretRotator.getSelectedSensorPosition();
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
        _flywheelClosedLoopController.setSetpoint(velocity.in(RotationsPerSecond), ControlType.kVelocity);
    }

    @Override
    public void controlYawAngle(Angle angle) {
        _turretRotator.set(TalonSRXControlMode.Position, getRotatorEncoderTicksFromTurretYaw(angle));
    }

    @Override
    public void setYawPercentOut(double percentOut) {
        _turretRotator.set(TalonSRXControlMode.PercentOutput, percentOut);
    }

    @Override
    public void controlHood(Angle angle) {
        _hoodClosedLoopController.setSetpoint(angle.in(Degrees), ControlType.kPosition);
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
        double busVoltage = _turretRotator.getBusVoltage();
        double percentOutput = volts / busVoltage;

        _turretRotator.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setHoodPercentOut(double percentOut) {
        _sparkHood.set(percentOut);
    }
}
