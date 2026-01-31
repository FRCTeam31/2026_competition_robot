package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;
import org.prime.control.ExtendedPIDConstants;

public class ClimbReal implements IClimb {

    private TalonFX _climbMotor;
    private DoubleSolenoid _frictionBrakeSolenoid;

    public ClimbReal() {
        configureClimbMotor(ClimbMap.CLIMB_MOTOR_PID);
        configureSupportMotor(ClimbMap.SUPPORT_MOTOR_PID);

        _frictionBrakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), ClimbMap.FrictionBrakeForwardChannel,
                ClimbMap.FrictionBrakeReverseChannel);
    }

    public void configureClimbMotor(ExtendedPIDConstants pid) {
        _climbMotor = new TalonFX(ClimbMap.CLIMB_MOTOR_CANID);
        TalonFXConfiguration config = new TalonFXConfiguration();

        // TODO: Configure climb motor

        config.MotorOutput.Inverted = ClimbMap.CLIMB_MOTOR_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80; // TODO: Determine
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60; // TODO: Determine

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.Feedback.SensorToMechanismRatio = ClimbMap.CLIMB_MOTOR_GEAR_RATIO;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = pid.kP;
        slot0.kI = pid.kI;
        slot0.kD = pid.kD;
        slot0.kS = pid.kS;
        slot0.kV = pid.kV;
        slot0.kA = pid.kA;
        config.Slot0 = slot0;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ClimbMap.CLIMB_MOTOR_RAMP_PERIOD;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;

        _climbMotor.getConfigurator().apply(config);
        _climbMotor.clearStickyFaults();
    }

    public void configureSupportMotor(ExtendedPIDConstants pid) {
        // TODO: Implement once CAD decides on the motor we're using
    }

    @Override
    public void updateInputs(ClimbInputsAutoLogged inputs) {
        controlClimb(inputs.climbState);
        controlFrictionBrake(inputs.frictionBrakeState);
    }

    @Override
    public void controlClimb(Climb.ClimbState state) {
        switch (state) {
            case Up:
                _climbMotor.set(0.5);
                break;
            case Down:
                _climbMotor.set(-0.5);
                break;
            case Stopped:
            default:
                _climbMotor.set(0);
                break;
        }
    }

    @Override
    public void controlSupport(Climb.SupportState state) {
        // Control support here based on state
    }

    @Override
    public void controlFrictionBrake(Climb.FrictionBrakeState state) {
        switch (state) {
            case Applied:
                _frictionBrakeSolenoid.set(DoubleSolenoid.Value.kForward);
                break;
            case Released:
            default:
                _frictionBrakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
        }
    }


}
