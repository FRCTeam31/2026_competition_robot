package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.SwerveMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.ExecutorService;

import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.dashboard.DashboardSection;
import org.prime.util.SwerveUtil;

public class SwerveModuleReal implements ISwerveModule {
  private String _name;
  private SwerveModuleMap _map;
  private DashboardSection _dashboardSection;
  private final String _optimizeModuleKey = "Optimize";

  // Configuration Thread Setup
  private ExecutorService _executorService;
  private int _configurationAttempts = 5;
  private int _timeBetweenConfigurationAttemptsMs = 500;

  // Devices
  private TalonFX _steeringMotor;
  private TalonFX _driveMotor;
  private CANcoder _encoder;

  // Status signals
  private StatusSignal<AngularVelocity> _driveVelocity; // Velocity from integrated sensor in rotations per second
  private StatusSignal<Angle> _drivePosition; // Position from integrated sensor in rotations
  private StatusSignal<Voltage> _driveVoltage;
  private StatusSignal<Angle> _steeringAzimuth; // Absolute position from CANCoder in rotations, with offset applied
  private StatusSignal<Angle> _steeringPosition;

  // Control requests
  private final MotionMagicVoltage _steeringControl = new MotionMagicVoltage(0);
  private final VelocityVoltage _driveControl = new VelocityVoltage(0);

  public SwerveModuleReal(String name, SwerveModuleMap moduleMap, ExecutorService configurationService,
      CANBus canivore) {
    _name = name;
    _map = moduleMap;
    _executorService = configurationService;
    _dashboardSection = new DashboardSection("Drive/" + _name);
    _dashboardSection.putBoolean(_optimizeModuleKey, true);

    setupCanCoder(canivore);
    setupSteeringMotor(SwerveMap.SteeringPID, canivore);
    setupDriveMotor(SwerveMap.DrivePID, canivore);

    BaseStatusSignal.setUpdateFrequencyForAll(1000, _drivePosition, _driveVelocity, _steeringAzimuth);
    BaseStatusSignal.setUpdateFrequencyForAll(50, _driveVoltage, _steeringPosition);
    ParentDevice.optimizeBusUtilizationForAll(_driveMotor, _steeringMotor, _encoder);
  }

  /**
   * Configures the CANCoder first so it can be used as a remote sensor
   */
  private void setupCanCoder(CANBus canivore) {
    _encoder = new CANcoder(_map.CANCoderCanId, canivore);
    _encoder.clearStickyFaults();

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    canCoderConfig.MagnetSensor.MagnetOffset = -_map.CanCoderStartingOffset;

    _encoder.getConfigurator().apply(canCoderConfig);
    _encoder.clearStickyFaults();

    _steeringAzimuth = _encoder.getPosition(false);
  }

  /**
   * Configures the steering motor with Motion Magic and CANCoder as remote sensor
   */
  private void setupSteeringMotor(ExtendedPIDConstants pid, CANBus canivore) {
    _steeringMotor = new TalonFX(_map.SteeringMotorCanId, canivore);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Output Configuration
    config.MotorOutput.Inverted = _map.SteerInverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45;

    // Feedback Configuration - Use CANCoder as remote sensor
    config.Feedback.FeedbackRemoteSensorID = _map.CANCoderCanId;
    // TODO: Purchase Phoenix Pro to use Fused CANcoder
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
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
    motionMagic.MotionMagicCruiseVelocity = 15; // rotations per second
    motionMagic.MotionMagicAcceleration = 100; // rotations per second^2
    motionMagic.MotionMagicJerk = 1000; // rotations per second^3
    config.MotionMagic = motionMagic;

    // Closed Loop Configuration
    config.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for steering

    // Apply configuration
    applyConfig(_steeringMotor, config);
    _steeringMotor.clearStickyFaults();

    // Configure control request to use FOC and slot 0
    // _steeringControl.EnableFOC = true;
    _steeringControl.Slot = 0;

    // Initialize status signals
    _steeringPosition = _steeringMotor.getPosition(false);
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
  private void setupDriveMotor(ExtendedPIDConstants pid, CANBus canivore) {
    _driveMotor = new TalonFX(_map.DriveMotorCanId, canivore);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Output Configuration
    config.MotorOutput.Inverted = _map.DriveInverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;

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
    applyConfig(_driveMotor, config);
    _driveMotor.clearStickyFaults();

    // Configure control request to use FOC and slot 0
    _driveControl.EnableFOC = true;
    _driveControl.Slot = 0;

    // Initialize status signals
    _driveVelocity = _driveMotor.getVelocity(false);
    _drivePosition = _driveMotor.getPosition(false);
    _driveVoltage = _driveMotor.getMotorVoltage(false);
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

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    _executorService.submit(() -> {
      StatusCode status = StatusCode.StatusCodeNotInitialized;

      for (int i = 0; i < _configurationAttempts; i++) {
        status = motor.getConfigurator().apply(config);

        if (status.isOK()) {
          System.out.println("Config applied for motor " + motor.getDeviceID() + " successfully, shutting down thread");
          break;
        }

        System.out.println("Retrying config apply for motor " + motor.getDeviceID() + "... attempt " + (i + 1));

        try {
          Thread.sleep(_timeBetweenConfigurationAttemptsMs);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }

      if (!status.isOK()) {
        System.out.println("FAILED to apply config for motor " + motor.getDeviceID() + ": " + status);
      }
    });
  }

  @Override
  public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    // Refresh odometry-critical status signals to get the latest values from the hardware
    // CANIVore timesync pattern: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html#canivore-timesync
    BaseStatusSignal.waitForAll(0.010, _drivePosition, _driveVelocity, _steeringAzimuth);

    // Non-blocking refresh for telemetry-only signals
    BaseStatusSignal.refreshAll(_driveVoltage, _steeringPosition);

    var az = getCurrentAzimuth();
    inputs.ModuleState.angle = az;
    inputs.ModuleState.speedMetersPerSecond = getCurrentVelocity().in(MetersPerSecond);
    inputs.ModulePosition.angle = az;
    inputs.ModulePosition.distanceMeters = getModuleDistance().in(Meters);
    inputs.DriveMotorVoltage = _driveVoltage.getValue().in(Volts);

    Logger.recordOutput("Swerve/Modules/" + _name + "/SteeringMotorPosition",
        _steeringPosition.getValue().in(Rotations));
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

    var currentHeading = getCurrentAzimuth();
    if (optimize) {
      desiredState = SwerveUtil.optimize(desiredState, currentHeading);
    }

    // Scale speed by cosine of angle error for smoother driving
    desiredState.cosineScale(currentHeading);

    // Set the drive and steering motors to the desired state
    setDriveSpeed(desiredState.speedMetersPerSecond);
    setModuleAngle(desiredState.angle);
  }

  private void setDriveSpeed(double desiredSpeedMetersPerSecond) {
    // Convert speed from meters per second to rotations per second
    // The motor controller will handle this based on SensorToMechanismRatio
    var wheelRotationsPerSecond = desiredSpeedMetersPerSecond / SwerveMap.DriveWheelCircumferenceMeters;

    Logger.recordOutput("Swerve/Modules/" + _name + "/DesiredWheelRPS", wheelRotationsPerSecond);
    // Send velocity command in rotations per second (wheel rotations)
    _driveMotor.setControl(_driveControl.withVelocity(wheelRotationsPerSecond));
  }

  private void setModuleAngle(Rotation2d angle) {
    // Get target position in rotations
    var targetRotations = angle.getRotations();

    Logger.recordOutput("Swerve/Modules/" + _name + "/TargetAngle", angle.getDegrees());

    // Send Motion Magic position command in rotations
    _steeringMotor.setControl(_steeringControl.withPosition(targetRotations));
  }

  /**
   * Gets the current heading of the module from the CANCoder
   */
  private Rotation2d getCurrentAzimuth() {
    return Rotation2d.fromRotations(_steeringAzimuth.getValue().in(Rotations));
  }

  /**
   * Gets the current velocity of the module
   */
  private MutLinearVelocity getCurrentVelocity() {
    // Get velocity in wheel rotations per second from the motor
    var wheelRPS = _driveVelocity.getValue().in(RotationsPerSecond);

    // Convert to meters per second
    var speedMps = wheelRPS * SwerveMap.DriveWheelCircumferenceMeters;

    return Units.MetersPerSecond.mutable(speedMps);
  }

  /**
   * Gets the distance the module has traveled
   */
  private MutDistance getModuleDistance() {
    // Get position in wheel rotations from the motor
    var wheelRotations = -_drivePosition.getValue().in(Rotations);

    // Convert to meters
    var distMeters = wheelRotations * SwerveMap.DriveWheelCircumferenceMeters;

    return Meters.mutable(distMeters);
  }
}