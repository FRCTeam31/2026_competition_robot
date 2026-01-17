package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import frc.robot.dashboard.DashboardSection;
import frc.robot.subsystems.swerve.SwerveMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.SwerveUtil;

public class SwerveModuleReal implements ISwerveModule {
  private String _name;
  private SwerveModuleMap _map;
  private DashboardSection _dashboardSection;
  private final String _optimizeModuleKey = "Optimize";

  // Devices
  private TalonFX _steeringMotor;
  private TalonFX _driveMotor;
  private CANcoder _encoder;

  // Control requests
  private final MotionMagicVoltage _steeringControl = new MotionMagicVoltage(0);
  private final VelocityVoltage _driveControl = new VelocityVoltage(0);

  public SwerveModuleReal(String name, SwerveModuleMap moduleMap) {
    _name = name;
    _map = moduleMap;
    _dashboardSection = new DashboardSection("Drive/" + _name);
    _dashboardSection.putBoolean(_optimizeModuleKey, true);

    setupCanCoder();
    setupSteeringMotor(SwerveMap.SteeringPID);
    setupDriveMotor(SwerveMap.DrivePID);
  }

  /**
   * Configures the CANCoder first so it can be used as a remote sensor
   */
  private void setupCanCoder() {
    _encoder = new CANcoder(_map.CANCoderCanId);
    _encoder.clearStickyFaults();

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    canCoderConfig.MagnetSensor.MagnetOffset = -_map.CanCoderStartingOffset;

    _encoder.getConfigurator().apply(canCoderConfig);
  }

  /**
   * Configures the steering motor with Motion Magic and CANCoder as remote sensor
   */
  private void setupSteeringMotor(ExtendedPIDConstants pid) {
    _steeringMotor = new TalonFX(_map.SteeringMotorCanId);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Output Configuration
    config.MotorOutput.Inverted = _map.SteerInverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    // Feedback Configuration - Use CANCoder as remote sensor
    config.Feedback.FeedbackRemoteSensorID = _map.CANCoderCanId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.RotorToSensorRatio = SwerveMap.SteeringGearRatio;
    config.Feedback.SensorToMechanismRatio = 1.0;

    // PID Configuration (Slot 0)
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = pid.kP;
    slot0.kI = pid.kI;
    slot0.kD = pid.kD;
    slot0.kS = pid.kS; // Static friction feedforward
    slot0.kV = pid.kV; // Velocity feedforward
    slot0.kA = pid.kA; // Acceleration feedforward
    config.Slot0 = slot0;

    // Motion Magic Configuration
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicCruiseVelocity = 100; // rotations per second (tune this)
    motionMagic.MotionMagicAcceleration = 200; // rotations per second^2 (tune this)
    motionMagic.MotionMagicJerk = 1600; // rotations per second^3 (tune this)
    config.MotionMagic = motionMagic;

    // Closed Loop Configuration
    config.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for steering

    // Apply configuration
    _steeringMotor.getConfigurator().apply(config);
    _steeringMotor.clearStickyFaults();

    // Configure control request to use FOC and slot 0
    _steeringControl.EnableFOC = true;
    _steeringControl.Slot = 0;
  }

  @Override
  public void setSteeringPID(ExtendedPIDConstants steeringPID) {
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = steeringPID.kP;
    slot0.kI = steeringPID.kI;
    slot0.kD = steeringPID.kD;
    slot0.kS = steeringPID.kS;
    slot0.kV = steeringPID.kV;
    slot0.kA = steeringPID.kA;

    _steeringMotor.getConfigurator().apply(slot0);
    System.out.println("Reset Steering PID " + _name);
  }

  /**
   * Configures the drive motor with velocity PID control
   */
  private void setupDriveMotor(ExtendedPIDConstants pid) {
    _driveMotor = new TalonFX(_map.DriveMotorCanId);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Output Configuration
    config.MotorOutput.Inverted = _map.DriveInverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    // Voltage Configuration
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;

    // Feedback Configuration - Use integrated sensor
    config.Feedback.SensorToMechanismRatio = SwerveMap.DriveGearRatio;

    // PID Configuration (Slot 0)
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = pid.kP;
    slot0.kI = pid.kI;
    slot0.kD = pid.kD;
    slot0.kS = pid.kS; // Static friction feedforward
    slot0.kV = pid.kV; // Velocity feedforward
    slot0.kA = pid.kA; // Acceleration feedforward
    config.Slot0 = slot0;

    // Open Loop Ramps
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = _map.DriveMotorRampRate;

    // Disable hardware limit switches
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    // Apply configuration
    _driveMotor.getConfigurator().apply(config);
    _driveMotor.clearStickyFaults();

    // Configure control request to use FOC and slot 0
    _driveControl.EnableFOC = true;
    _driveControl.Slot = 0;
  }

  @Override
  public void setDrivePID(ExtendedPIDConstants drivePID) {
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = drivePID.kP;
    slot0.kI = drivePID.kI;
    slot0.kD = drivePID.kD;
    slot0.kS = drivePID.kS;
    slot0.kV = drivePID.kV;
    slot0.kA = drivePID.kA;

    _driveMotor.getConfigurator().apply(slot0);
    System.out.println("Reset Drive PID " + _name);
  }

  @Override
  public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    var rotation = getCurrentHeading();
    var speedMps = getCurrentVelocity().in(MetersPerSecond);
    var distanceMeters = getModuleDistance();

    inputs.ModuleState.angle = rotation;
    inputs.ModuleState.speedMetersPerSecond = speedMps;
    inputs.ModulePosition.angle = rotation;
    inputs.ModulePosition.distanceMeters = distanceMeters.magnitude();
    inputs.DriveMotorVoltage = _driveMotor.getMotorVoltage().getValueAsDouble();

    Logger.recordOutput("Swerve/Modules/" + _name + "/DriveMotorMeasuredVoltage",
        _driveMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Swerve/Modules/" + _name + "/SteeringMotorPosition",
        _steeringMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Swerve/Modules/" + _name + "/CANCoderPosition",
        _encoder.getPosition().getValueAsDouble());
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    _driveMotor.setVoltage(voltage);
    setModuleAngle(moduleAngle);
  }

  @Override
  public void stopMotors() {
    _driveMotor.stopMotor();
    _steeringMotor.stopMotor();
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The optimized state of the module that we'd like to be at in this period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    var optimize = _dashboardSection.getBoolean(_optimizeModuleKey, true);
    Logger.recordOutput("Swerve/Modules/" + _name + "/Optimized", optimize);
    if (optimize) {
      desiredState = SwerveUtil.optimize(desiredState, getCurrentHeading());
    }

    // Scale speed by cosine of angle error for smoother driving
    desiredState.cosineScale(getCurrentHeading());

    // Set the drive and steering motors to the desired state
    setDriveSpeed(desiredState.speedMetersPerSecond);
    setModuleAngle(desiredState.angle);
  }

  private void setDriveSpeed(double desiredSpeedMetersPerSecond) {
    // Convert speed from meters per second to rotations per second
    // The motor controller will handle this based on SensorToMechanismRatio
    var wheelRotationsPerSecond = desiredSpeedMetersPerSecond / SwerveMap.DriveWheelCircumferenceMeters;

    Logger.recordOutput("Swerve/Modules/" + _name + "/DesiredWheelRPS", wheelRotationsPerSecond);
    Logger.recordOutput("Swerve/Modules/" + _name + "/ActualWheelRPS",
        getCurrentVelocity().in(MetersPerSecond) / SwerveMap.DriveWheelCircumferenceMeters);

    // Send velocity command in rotations per second (wheel rotations)
    _driveMotor.setControl(_driveControl.withVelocity(wheelRotationsPerSecond));
  }

  private void setModuleAngle(Rotation2d angle) {
    // Get target position in rotations
    var targetRotations = angle.getRotations();

    Logger.recordOutput("Swerve/Modules/" + _name + "/TargetAngle", angle.getDegrees());
    Logger.recordOutput("Swerve/Modules/" + _name + "/CurrentAngle", getCurrentHeading().getDegrees());

    // Send Motion Magic position command in rotations
    _steeringMotor.setControl(_steeringControl.withPosition(targetRotations));
  }

  /**
   * Gets the current heading of the module from the CANCoder
   */
  private Rotation2d getCurrentHeading() {
    return Rotation2d.fromRotations(_encoder.getPosition().getValueAsDouble());
  }

  /**
   * Gets the current velocity of the module
   */
  private MutLinearVelocity getCurrentVelocity() {
    // Get velocity in wheel rotations per second from the motor
    var wheelRPS = _driveMotor.getVelocity().getValueAsDouble();

    // Convert to meters per second
    var speedMps = wheelRPS * SwerveMap.DriveWheelCircumferenceMeters;

    return Units.MetersPerSecond.mutable(speedMps);
  }

  /**
   * Gets the distance the module has traveled
   */
  private MutDistance getModuleDistance() {
    // Get position in wheel rotations from the motor
    var wheelRotations = _driveMotor.getPosition().getValueAsDouble();

    // Convert to meters
    var distMeters = wheelRotations * SwerveMap.DriveWheelCircumferenceMeters;

    return Meters.mutable(distMeters);
  }
}