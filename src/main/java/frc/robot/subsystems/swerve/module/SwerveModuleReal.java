package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
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
  private CANcoder _encoder;

  public SwerveModuleReal(String name, SwerveModuleMap moduleMap) {
    _name = name;
    _map = moduleMap;
    _dashboardSection = new DashboardSection("Drive/" + _name);
    _dashboardSection.putBoolean(_optimizeModuleKey, true);

    setupSteeringMotor(SwerveMap.SteeringPID);
    setupDriveMotor(SwerveMap.DrivePID);
    setupCanCoder();
  }

  /**
   * Configures the steering motor and PID controller
   */
  private void setupSteeringMotor(ExtendedPIDConstants pid) {
    // TODO
  }

  @Override
  public void setSteeringPID(ExtendedPIDConstants steeringPID) {
    // TODO
    System.out.println("Reset Steering PID " + _name);
  }

  /**
   * Configures the drive motors
   * 
   * @param pid
   */
  private void setupDriveMotor(ExtendedPIDConstants pid) {
    // TODO
  }

  @Override
  public void setDrivePID(ExtendedPIDConstants drivePID) {
    // TODO
    System.out.println("Reset Drive PID " + _name);
  }

  /**
   * Configures the CANCoder
   */
  private void setupCanCoder() {
    _encoder = new CANcoder(_map.CANCoderCanId);
    _encoder.clearStickyFaults();
    _encoder.getConfigurator().apply(new CANcoderConfiguration());

    // AbsoluteSensorRangeValue
    _encoder.getConfigurator()
        .apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(-_map.CanCoderStartingOffset)));
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
    inputs.DriveMotorVoltage = 0; // TODO
  }

  @Override
  public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
    // TODO

    setModuleAngle(moduleAngle);
  }

  @Override
  public void stopMotors() {
    // TODO
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

    // Scale speed by cosine of angle error for smoother driving.
    desiredState.cosineScale(getCurrentHeading());

    // Set the drive motor to the desired speed
    setDriveSpeed(desiredState.speedMetersPerSecond);

    // Set the steering motor to the desired angle, if trying to drive
    if (Math.abs(desiredState.speedMetersPerSecond) > 0.05) {
      setModuleAngle(desiredState.angle);
    }
  }

  private void setDriveSpeed(double desiredSpeedMetersPerSecond) {
    // TODO
  }

  private void setModuleAngle(Rotation2d angle) {
    // Normalize to 0 to 1
    var setpoint = angle.getRotations() % 1;
    if (setpoint < 0)
      setpoint += 1;

    // TODO
  }

  /**
   * Gets the current heading of the module
   */
  private Rotation2d getCurrentHeading() {
    return Rotation2d.fromRotations(_encoder.getPosition().getValueAsDouble());
  }

  /**
   * Gets the current velocity of the module
   */
  private MutLinearVelocity getCurrentVelocity() {
    var speedMps = 0; // TODO

    return Units.MetersPerSecond.mutable(speedMps);
  }

  /**
   * Gets the distance the module has traveled
   */
  private MutDistance getModuleDistance() {
    var distMeters = 0; // TODO

    return Meters.mutable(distMeters);
  }
}
