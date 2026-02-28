package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;

public class ClimbReal implements IClimb {

    // Devices
    private SparkFlex _climbMotor;
    private DoubleSolenoid _frictionBrakeSolenoid;
    private DoubleSolenoid _supportSolenoid;

    private DigitalInput _limitSwitch;

    public ClimbReal() {
        configureClimbMotor();

        _frictionBrakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.FrictionBrakeForwardChannel,
                ClimbMap.FrictionBrakeReverseChannel);

        _supportSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.SUPPORT_FORWARD_CHANNEL,
                ClimbMap.SUPPORT_REVERSE_CHANNEL);

        _limitSwitch = new DigitalInput(ClimbMap.LIMIT_SWITCH_CHANNEL);
    }

    public void configureClimbMotor() {
        _climbMotor = new SparkFlex(ClimbMap.CLIMB_MOTOR_CANID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(ClimbMap.CLIMB_MOTOR_INVERTED);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(70, 60);

        config.closedLoopRampRate(ClimbMap.CLIMB_MOTOR_RAMP_PERIOD);

        _climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _climbMotor.clearFaults();
    }

    @Override
    public void updateInputs(ClimbInputsAutoLogged inputs) {
        inputs.loweredLimitSwitch = _limitSwitch.get();
        inputs.climberExtension = ClimbMap.CLIMB_PULLEY_RADIUS
                .times(_climbMotor.getEncoder().getPosition() * Math.PI * 2 / ClimbMap.CLIMB_MOTOR_GEAR_RATIO);
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

    @Override
    public void zeroEncoder() {
        _climbMotor.getEncoder().setPosition(0);
    }

}
