package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Container;
import frc.robot.FieldTargets;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import frc.robot.subsystems.swerve.util.AutoAlign;
import frc.robot.subsystems.vision.VisionMap;
import frc.robot.subsystems.vision.limelight.LimelightCameraInputsAutoLogged;
import frc.robot.subsystems.vision.photon.PhotonCameraInputsAutoLogged;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.function.Supplier;

import org.prime.control.ImpactRumbleHelper;
import org.prime.control.PrimeHolonomicDriveController;
import org.prime.control.SwerveControlSuppliers;
import org.prime.dashboard.Elastic;
import org.prime.dashboard.Elastic.Notification;
import org.prime.dashboard.Elastic.NotificationLevel;
import org.prime.subsystems.LoggedSubsystem;
import org.prime.sysid.SysIdRoutineHelper;

public class Swerve extends LoggedSubsystem {

  private ImpactRumbleHelper _rumbleHelper;

  // IO
  private SwerveIOPackager _swervePackager;

  // AutoAlign & Pathfinding
  private AutoAlign _autoAlign;
  private Command _activePathfindCommand;

  // Vision, Kinematics, odometry
  private PrimeHolonomicDriveController _primeHolonomicController;
  private RobotConfig _pathplannerRobotConfig;

  // SysId characterization
  private final SysIdRoutineHelper _driveSysId;

  private Alert _photonTooFarAlert = new Alert("Photon estimation skipped", AlertType.kInfo);

  /**
   * Creates a new Drivetrain.
   */
  public Swerve() {
    setName("Swerve");

    _rumbleHelper = new ImpactRumbleHelper();

    // Create swerve controller
    _swervePackager = new SwerveIOPackager();
    _swervePackager.updateInputs(SuperStructure.Swerve);

    // Configure AutoAlign
    _autoAlign = new AutoAlign(SwerveMap.AutoAlignPID);

    configurePathPlanner();

    // Configure SysId routine for drive motor characterization.
    // All 4 modules are driven in unison, locked at 0-degrees heading.
    _driveSysId = new SysIdRoutineHelper(
        this,
        "SwerveDrive",
        (voltage) -> _swervePackager.setDriveVoltageAllModules(
            voltage.in(edu.wpi.first.units.Units.Volts), Rotation2d.fromDegrees(0)),
        (log) -> {
          var states = SuperStructure.Swerve.ModuleStates;
          var positions = _swervePackager.getModulePositions();

          log.motor("drive-fl")
              .voltage(edu.wpi.first.units.Units.Volts.of(SuperStructure.SwerveModules[0].DriveMotorVoltage))
              .linearVelocity(MetersPerSecond.of(states[0].speedMetersPerSecond))
              .linearPosition(Meters.of(positions[0].distanceMeters));
          log.motor("drive-fr")
              .voltage(edu.wpi.first.units.Units.Volts.of(SuperStructure.SwerveModules[1].DriveMotorVoltage))
              .linearVelocity(MetersPerSecond.of(states[1].speedMetersPerSecond))
              .linearPosition(Meters.of(positions[1].distanceMeters));
          log.motor("drive-rl")
              .voltage(edu.wpi.first.units.Units.Volts.of(SuperStructure.SwerveModules[2].DriveMotorVoltage))
              .linearVelocity(MetersPerSecond.of(states[2].speedMetersPerSecond))
              .linearPosition(Meters.of(positions[2].distanceMeters));
          log.motor("drive-rr")
              .voltage(edu.wpi.first.units.Units.Volts.of(SuperStructure.SwerveModules[3].DriveMotorVoltage))
              .linearVelocity(MetersPerSecond.of(states[3].speedMetersPerSecond))
              .linearPosition(Meters.of(positions[3].distanceMeters));
        });
  }

  private void configurePathPlanner() {
    // Load the RobotConfig from the GUI settings, or use the default if an exception occurs
    _pathplannerRobotConfig = SwerveMap.PathPlannerRobotConfiguration;
    try {
      _pathplannerRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Set up PP to feed current path poses to the dashboard's field widget
    // PathPlannerLogging.setLogCurrentPoseCallback(pose -> Container.TeleopDashboardSection.setFieldRobotPose(pose));
    PathPlannerLogging
        .setLogTargetPoseCallback(pose -> Container.Dashboard.getFieldTargetPose().setPose(pose));
    PathPlannerLogging
        .setLogActivePathCallback(poses -> Container.Dashboard.getFieldPath().setPoses(poses));

    // Configure PathPlanner holonomic control
    _primeHolonomicController = new PrimeHolonomicDriveController(
        SwerveMap.PathPlannerTranslationPID.toPIDConstants(),
        SwerveMap.PathPlannerRotationPID.toPIDConstants());
    if (!AutoBuilder.isConfigured()) {
      AutoBuilder.configure(
          () -> SuperStructure.Swerve.EstimatedRobotPose,
          _swervePackager::setEstimatorPose,
          () -> SuperStructure.Swerve.RobotRelativeChassisSpeeds,
          (speeds, feedForwards) -> driveRobotRelative(speeds),
          _primeHolonomicController,
          _pathplannerRobotConfig,
          Robot::onRedAlliance, // Boolean supplier that controls when the path will be mirrored for the red alliance
          this);
    }

    // Override PathPlanner's rotation feedback
    // PPHolonomicDriveController.overrideRotationFeedback(() -> SuperStructure.SwerveState.AutoAlignCorrection);
  }

  // #region Control methods

  /**
   * Resets the gyro angle
   */
  public void resetGyro() {
    _swervePackager.resetGyro();
  }

  /**
   * Enabled/disables AutoAlign control. Also overrides PathPlanner's rotation, if enabled
   */
  private void setAutoAlignEnabled(boolean enabled) {
    SuperStructure.Swerve.UseAutoAlign = enabled;
    if (enabled) {
      PPHolonomicDriveController.overrideRotationFeedback(() -> SuperStructure.Swerve.AutoAlignCorrection);
    } else {
      PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * 
   * @param robotRelativeChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeChassisSpeeds) {
    // If AutoAlign is enabled, override the input rotational speed to reach the setpoint
    recordOutput("autoAlignCorrection", SuperStructure.Swerve.AutoAlignCorrection);

    robotRelativeChassisSpeeds.omegaRadiansPerSecond = SuperStructure.Swerve.UseAutoAlign
        ? SuperStructure.Swerve.AutoAlignCorrection
        : -robotRelativeChassisSpeeds.omegaRadiansPerSecond;

    // Correct drift by taking the input speeds and converting them to a desired
    // per-period speed. This is known as "discretizing"
    robotRelativeChassisSpeeds = ChassisSpeeds.discretize(robotRelativeChassisSpeeds, 0.02);
    recordOutput("desiredChassisSpeeds", robotRelativeChassisSpeeds);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = _swervePackager.Kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveMap.Chassis.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    recordOutput("desiredStates", swerveModuleStates);
    _swervePackager.setDesiredModuleStates(swerveModuleStates);

    // Update the gyro omega for simulation purposes
    _swervePackager.setSimGyroOmega(robotRelativeChassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Processes vision estimations when within a certain velocity threshold
   */
  @SuppressWarnings("unused")
  private void processVisionEstimations() {
    // (1 rad/s is about 60 degrees/s)
    var currentRotationalVelocity = RadiansPerSecond
        .of(Math.abs(SuperStructure.Swerve.RobotRelativeChassisSpeeds.omegaRadiansPerSecond)).abs(RadiansPerSecond);
    var currentXVelocity = MetersPerSecond.of(SuperStructure.Swerve.RobotRelativeChassisSpeeds.vxMetersPerSecond)
        .abs(MetersPerSecond);
    var currentYVelocity = MetersPerSecond.of(SuperStructure.Swerve.RobotRelativeChassisSpeeds.vyMetersPerSecond)
        .abs(MetersPerSecond);

    var withinPoseEstimationVelocity = currentRotationalVelocity < 60 &&
        currentXVelocity < 2 &&
        currentYVelocity < 2;

    recordOutput("withinPoseEstimationVelocity", withinPoseEstimationVelocity);
    if (!withinPoseEstimationVelocity) {
      return;
    }

    if (SuperStructure.VisionLimelights.containsKey(VisionMap.LimelightTurretName)
        && SwerveMap.USE_LIMELIGHT_POSE_ESTIMATION) {
      evaluateLimelightPoseEstimation(SuperStructure.VisionLimelights.get(VisionMap.LimelightTurretName));
    }

    if (SuperStructure.VisionPhotons.containsKey(VisionMap.PhotonCam1Name)) {
      evaluatePhotonPoseEstimation(SuperStructure.VisionPhotons.get(VisionMap.PhotonCam1Name));
    }

    if (SuperStructure.VisionPhotons.containsKey(VisionMap.PhotonCam2Name)) {
      evaluatePhotonPoseEstimation(SuperStructure.VisionPhotons.get(VisionMap.PhotonCam2Name));
    }
  }

  /**
   * Evaluates a limelight pose and feeds it into the pose estimator
   */
  private void evaluateLimelightPoseEstimation(LimelightCameraInputsAutoLogged llInputs) {
    // If no tags in view, reject the update
    if (llInputs.BotPoseEstimate == null || llInputs.BotPoseEstimate.tagCount == 0)
      return;

    if (llInputs.BotPoseEstimate.tagCount == 1 && llInputs.BotPoseEstimate.rawFiducials.length == 1) {
      var target = llInputs.BotPoseEstimate.rawFiducials[0].id;
      boolean isValidTarget = target >= 1 && target <= 22;
      boolean tooAmbiguous = llInputs.BotPoseEstimate.rawFiducials[0].ambiguity > .7;
      boolean tooFar = llInputs.BotPoseEstimate.rawFiducials[0].distToCamera > 3;

      // If the tag is not valid, too ambiguous, or too far, reject the update
      if (!isValidTarget || tooAmbiguous || tooFar)
        return;
    }

    // If we've made it this far, we can trust the pose estimate
    _swervePackager.addPoseEstimatorVisionMeasurement(
        llInputs.BotPoseEstimate.pose,
        llInputs.BotPoseEstimate.timestampSeconds,
        VecBuilder.fill(.5, .5, 9999999));
  }

  private void evaluatePhotonPoseEstimation(PhotonCameraInputsAutoLogged photonInputs) {
    // If no targets in view, reject the update
    if (photonInputs.BotPoseEstimate == null || photonInputs.TargetCount == 0)
      return;

    if (photonInputs.AverageTagDistance > VisionMap.PHOTON_MAX_AVG_TAG_DISTANCE_METERS) {
      System.out.println(photonInputs.AverageTagDistance);
      _photonTooFarAlert.set(true);
      return;
    }
    _photonTooFarAlert.set(false);

    _swervePackager.addPoseEstimatorVisionMeasurement(
        photonInputs.BotPoseEstimate,
        photonInputs.TimestampSeconds,
        VecBuilder.fill(
            photonInputs.CurrentStdDevs[0],
            photonInputs.CurrentStdDevs[1],
            photonInputs.CurrentStdDevs[2]));
  }

  // #endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Get inputs
    _swervePackager.updateInputs(SuperStructure.Swerve);

    SuperStructure.Swerve.AutoAlignCorrection = _autoAlign.getCorrection(SuperStructure.Swerve.GyroAngle);
    processInputs(SuperStructure.Swerve);

    processVisionEstimations();
    Container.Dashboard.setFieldRobotPose(SuperStructure.Swerve.EstimatedRobotPose);

    // Update LEDs
    recordOutput("autoAlign/Enabled", SuperStructure.Swerve.UseAutoAlign);
    recordOutput("autoAlign/Setpoint", _autoAlign.getSetpoint());
    recordOutput("autoAlign/AtSetpoint", _autoAlign.atSetpoint());

    if (DriverStation.isAutonomousEnabled()) {
      recordOutput("pp-translation-error", _primeHolonomicController.getTranslationError());
    }

    // Update rumble
    _rumbleHelper.addSample(
        SuperStructure.Swerve.GyroAccelX,
        SuperStructure.Swerve.GyroAccelY,
        SuperStructure.Swerve.GyroAccelZ,
        SuperStructure.Swerve.RobotRelativeChassisSpeeds.vxMetersPerSecond,
        SwerveMap.Chassis.MaxSpeedMetersPerSecond);
    Container.OperatorInterface.setControllerRumbleIntensity(Container.OperatorInterface.DriverController,
        _rumbleHelper.getRumbleIntensity());
  }

  // #region Commands

  /**
   * Creates a command that drives the robot in field-relative mode using input controls
   * @param controlSuppliers Controller input suppliers
   */
  public Command driveFieldRelativeCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
      var speeds = controlSuppliers.getChassisSpeeds(
          false,
          SuperStructure.Swerve.GyroAngle,
          () -> setAutoAlignEnabled(false));

      // If the driver is trying to move and we have an active pathfinding command, cancel it.
      var driverIsTryingToMove = speeds.omegaRadiansPerSecond > 0
          || speeds.vxMetersPerSecond > 0
          || speeds.vyMetersPerSecond > 0;
      if (driverIsTryingToMove) {
        var pathfindCommandIsCancellable = _activePathfindCommand != null
            && _activePathfindCommand.isScheduled()
            && !_activePathfindCommand.isFinished();

        if (pathfindCommandIsCancellable) {
          _activePathfindCommand.cancel();
        }
      }

      driveRobotRelative(speeds);
    }).handleInterrupt(() -> DriverStation.reportWarning("[DRIVE] Default command interrupted", false));
  }

  public Command cancelPathfindingCommand() {
    return Commands.runOnce(() -> {
      // Cancel the command if it's already running
      if (_activePathfindCommand != null && _activePathfindCommand.isScheduled()
          && !_activePathfindCommand.isFinished()) {
        _activePathfindCommand.cancel();
      }
    });
  }

  /**
   * Command for stopping all motors
   */
  public Command stopAllMotorsCommand() {
    return this.runOnce(_swervePackager::stopAllMotors);
  }

  /**
   * Command for resetting the gyro
   */
  public Command resetGyroCommand() {
    return Commands.runOnce(_swervePackager::resetGyro);
  }

  /**
   * Enables AutoAlign control and sets an angle setpoint
   * 
   * @param angle
   */
  public Command setAutoAlignSetpointCommand(double angle) {
    return Commands.runOnce(() -> {
      var setpoint = Robot.onBlueAlliance()
          ? angle + 180
          : angle;
      _autoAlign.setSetpoint(Rotation2d.fromDegrees(setpoint));
      SuperStructure.Swerve.AutoAlignSetpoint = setpoint;
      setAutoAlignEnabled(true);
    });
  }

  /**
   * Disables AutoAlign control
   */
  public Command disableAutoAlignCommand() {
    var cmd = Commands.runOnce(() -> setAutoAlignEnabled(false)).ignoringDisable(true);
    cmd.setName("DisableAutoAlign");

    return cmd;
  }

  /**
   * Enables AutoAlign for PathPlanner routines
   * @return
   */
  public Command enablePathPlannerAutoAlignRotationFeedbackCommand() {
    var cmd = Commands.runOnce(() -> {
      PPHolonomicDriveController.overrideRotationFeedback(() -> SuperStructure.Swerve.AutoAlignCorrection);
    });
    cmd.setName("EnableAutoAlignRotationFeedback");

    return cmd;
  }

  /**
   * Disables AutoAlign for PathPlanner routines
   * @return
   */
  public Command disablePathPlannerAutoAlignRotationFeedbackCommand() {
    var cmd = Commands.runOnce(PPHolonomicDriveController::clearRotationFeedbackOverride);
    cmd.setName("DisableAutoAlignRotationFeedback");

    return cmd;
  }

  /**
   * Creates a command which pathfinds to a given pose, flipping the path across the field center if desired
   * @param poseSupplier A supplier for the target pose
   */
  public Command pathfindToPoseCommand(Supplier<Pose2d> poseSupplier, boolean flipped) {
    return this.defer(() -> {
      var pathConstraints = new PathConstraints(SwerveMap.Chassis.MaxSpeedMetersPerSecond,
          SwerveMap.Chassis.MaxSpeedMetersPerSecond,
          SwerveMap.Chassis.MaxAngularSpeedRadians,
          SwerveMap.Chassis.MaxAngularSpeedRadians);

      var desiredPose = poseSupplier.get();

      return flipped
          ? AutoBuilder.pathfindToPoseFlipped(desiredPose, pathConstraints)
          : AutoBuilder.pathfindToPose(desiredPose, pathConstraints);
    });
  }

  /**
   * Creates a command that enables AutoAlign to orient the robot's front (positive-x) away from the hub.
   * While active, the setpoint is continuously updated based on the robot's current position relative
   * to the alliance hub. When the command ends, AutoAlign is disabled.
   */
  public Command faceAwayFromHubCommand() {
    return this.run(() -> {
      var hubPosition = FieldTargets.GetCurrentAllianceHubPosition();
      var robotPose = SuperStructure.Swerve.EstimatedRobotPose;

      // Calculate the field-centric angle from the robot to the hub
      double dx = hubPosition.getX() - robotPose.getX();
      double dy = hubPosition.getY() - robotPose.getY();
      double angleToHubRadians = Math.atan2(dy, dx);

      // Add pi to face AWAY from the hub
      var awayFromHubAngle = Rotation2d.fromRadians(angleToHubRadians + Math.PI);

      _autoAlign.setSetpoint(awayFromHubAngle);
      SuperStructure.Swerve.AutoAlignSetpoint = awayFromHubAngle.getDegrees();
      setAutoAlignEnabled(true);
    }).finallyDo(() -> setAutoAlignEnabled(false));
  }

  /*
   * Returns a map of named commands for the drivetrain subsystem for PathPlanner
   */
  public Map<String, Command> getNamedCommands() {
    return Map.of();
  }

  /**
   * Returns a SysId characterization command for all 4 swerve drive motors.
   * Modules are locked at 0-degrees heading and driven in unison.
   *
   * @param testType  QUASISTATIC (ramp) or DYNAMIC (step)
   * @param direction FORWARD or REVERSE
   * @return A command that runs the specified SysId test on the drive motors
   */
  public Command sysIdDriveCommand(SysIdRoutineHelper.TestType testType,
      SysIdRoutineHelper.TestDirection direction) {
    return _driveSysId.getCommand(testType, direction);
  }
  // #endregion
}
