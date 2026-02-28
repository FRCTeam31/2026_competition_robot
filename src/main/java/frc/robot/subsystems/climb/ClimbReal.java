package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;
import org.prime.control.ExtendedPIDConstants;

public class ClimbReal implements IClimb {

    //TODO: Convert to SparkFlex
    private SparkFlex _climbMotor;
    private DoubleSolenoid _frictionBrakeSolenoid;
    private DoubleSolenoid _supportSolenoid;

    private DigitalInput _upperLimitSwitch;
    private DigitalInput _lowerLimitSwitch;

    public ClimbReal() {
        configureClimbMotor(ClimbMap.CLIMB_MOTOR_PID);

        _frictionBrakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.FrictionBrakeForwardChannel,
                ClimbMap.FrictionBrakeReverseChannel);

        _supportSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.SUPPORT_FORWARD_CHANNEL,
                ClimbMap.SUPPORT_REVERSE_CHANNEL);

        _upperLimitSwitch = new DigitalInput(ClimbMap.UPPER_LIMIT_SWITCH_CHANNEL);
        _lowerLimitSwitch = new DigitalInput(ClimbMap.LOWER_LIMIT_SWITCH_CHANNEL);
    }

    public void configureClimbMotor(ExtendedPIDConstants pid) {
        _climbMotor = new SparkFlex(ClimbMap.CLIMB_MOTOR_CANID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(ClimbMap.CLIMB_MOTOR_INVERTED);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(70, 60);

        config.closedLoop.pid(pid.kP, pid.kI, pid.kD);

        config.closedLoopRampRate(ClimbMap.CLIMB_MOTOR_RAMP_PERIOD);

        _climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _climbMotor.clearFaults();
    }

    @Override
    public void updateInputs(ClimbInputsAutoLogged inputs) {
        inputs.upperLimitSwitch = _upperLimitSwitch.get();
        inputs.lowerLimitSwitch = _lowerLimitSwitch.get();
    }

    @Override
    public void controlClimb(double speed) {
        _climbMotor.set(speed);
    }

    @Override
    public void controlSupport(DoubleSolenoid.Value value) {
        _supportSolenoid.set(value);
    }

    @Override
    public void controlFrictionBrake(DoubleSolenoid.Value value) {
        _frictionBrakeSolenoid.set(value);
    }

}
