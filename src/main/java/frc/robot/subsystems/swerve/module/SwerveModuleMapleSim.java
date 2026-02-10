package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.dashboard.DashboardSection;
import frc.robot.subsystems.swerve.SwerveMap;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;
import org.prime.control.ExtendedPIDConstants;
import org.prime.util.SwerveUtil;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleMapleSim implements ISwerveModule {
    private String _name;
    private DashboardSection _dashboardSection;
    private final String _optimizeModuleKey = "Optimize";

    private final SwerveModuleSimulation _moduleSim;
    private final SimulatedMotorController.GenericMotorController _drive;
    private final SimulatedMotorController.GenericMotorController _steer;

    private PIDController _drivingPidController;
    private SimpleMotorFeedforward _driveFeedForward;
    private Rotation2d _currentHeading = new Rotation2d();

    public SwerveModuleMapleSim(String name, SwerveModuleSimulation moduleSim) {
        _name = name;
        _dashboardSection = new DashboardSection("Drivetrain/" + _name);
        _dashboardSection.putBoolean(_optimizeModuleKey, true);
        _moduleSim = moduleSim;

        _drive = _moduleSim
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(SwerveMap.DriveFreeCurrentLimit));
        _steer = _moduleSim
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(20));

        var pid = SwerveMap.DrivePID;
        _drivingPidController = pid.createPIDController(0.02);
        _driveFeedForward = new SimpleMotorFeedforward(pid.kS, pid.kV, pid.kA);
    }

    @Override
    public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        var speedMps = _moduleSim.getDriveWheelFinalSpeed().in(Units.RotationsPerSecond)
                * SwerveMap.DriveWheelCircumferenceMeters;

        inputs.ModuleState.angle = _currentHeading;
        inputs.ModuleState.speedMetersPerSecond = speedMps;
        inputs.ModulePosition.angle = _currentHeading;
        inputs.ModulePosition.distanceMeters = _moduleSim.getDriveWheelFinalPosition().in(Rotations)
                * SwerveMap.DriveWheelCircumferenceMeters;
        Logger.recordOutput("Swerve/Modules/" + _name + "/DriveMotorMeasuredVoltage", _drive.getAppliedVoltage());
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the desired state
        var optimize = _dashboardSection.getBoolean(_optimizeModuleKey, true);
        Logger.recordOutput("Swerve/Modules/" + _name + "/Optimized", optimize);
        if (optimize) {
            desiredState = SwerveUtil.optimize(desiredState, _currentHeading);
        }

        Logger.recordOutput("Swerve/Modules/" + _name + "/SteeringMotorOutputSpeed", 0);
        _currentHeading = desiredState.angle;

        // Set the drive motor to the desired speed
        // Calculate target data to voltage data
        var desiredSpeedRotationsPerSecond = (desiredState.speedMetersPerSecond / SwerveMap.DriveWheelCircumferenceMeters)
                * SwerveMap.DriveGearRatio;

        var ff = _driveFeedForward.calculate(desiredSpeedRotationsPerSecond);

        var currentSpeedRotationsPerSecond = _moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond) / (2 * Math.PI);
        var pid = _drivingPidController.calculate(currentSpeedRotationsPerSecond, desiredSpeedRotationsPerSecond);
        var driveOutput = MathUtil.clamp(ff + pid, -12.0, 12.0);

        Logger.recordOutput("Swerve/Modules/" + _name + "/DrivePID", pid);
        Logger.recordOutput("Swerve/Modules/" + _name + "/DriveFF", ff);
        Logger.recordOutput("Swerve/Modules/" + _name + "/DriveMotorOutputVoltage", driveOutput);
        _drive.requestVoltage(Voltage.ofBaseUnits(driveOutput, Volts));
    }

    @Override
    public void setDriveVoltage(double voltage, Rotation2d moduleAngle) {
        _drive.requestVoltage(Voltage.ofBaseUnits(voltage, Volts));
        _currentHeading = moduleAngle;
    }

    @Override
    public void stopMotors() {
        _drive.requestVoltage(Voltage.ofBaseUnits(0, Volts));
        _steer.requestVoltage(Voltage.ofBaseUnits(0, Volts));
    }

    @Override
    public void setDrivePID(ExtendedPIDConstants drivePID) {
        var currentSetpoint = _drivingPidController.getSetpoint();
        _drivingPidController = drivePID.createPIDController(0.02);
        _drivingPidController.setSetpoint(currentSetpoint);

        _driveFeedForward = new SimpleMotorFeedforward(drivePID.kS, drivePID.kV, drivePID.kA);
    }

    @Override
    public void setSteeringPID(ExtendedPIDConstants steeringPID) {
        // Not implemented
    }
}
