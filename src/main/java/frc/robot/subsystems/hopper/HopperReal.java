package frc.robot.subsystems.hopper;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _solenoid;
    private SparkFlex _feedSparkFlex;
    private SparkFlex _intakeControlSparkFlex;
    private SparkFlex _intakeFeedSparkFlex;
    private DigitalInput _inLimitSwitch;
    private DigitalInput _outLimitSwitch;

    public HopperReal() {
        _solenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.ForwardChannel,
                HopperMap.ReverseChannel);
        _feedSparkFlex = new SparkFlex(HopperMap.CANID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {
        inputs.intakeINLimitSwitch = _inLimitSwitch.get();
        inputs.intakeOUTLimitSwitch = _outLimitSwitch.get();
    }

    @Override
    public void setHopper(DoubleSolenoid.Value value) {
        _solenoid.set(value);
    }

    @Override
    public void toggleHopper() {
        _solenoid.toggle();
    }

    @Override
    public void setFeedSpeed(Hopper.FeedState feedState) {

        switch (feedState) {
            case INWARDS:
                _feedSparkFlex.set(0.5);
                break;
            case OUTWARDS:
                _feedSparkFlex.set(-0.5);
                break;
            case STOPPED:
            default:
                _feedSparkFlex.set(0);
                break;
        }
    }

    @Override
    public void feedStop() {
        _feedSparkFlex.stopMotor();
    }

    @Override
    public void setIntakePosition(Hopper.IntakeControlState controlState) {
        switch (controlState) {
            case OUT:
                if (_outLimitSwitch.get()) {
                    break;
                }
                _intakeControlSparkFlex.set(0.5);
                break;
            case IN:
            default:
                if (_inLimitSwitch.get()) {
                    break;
                }
                _intakeControlSparkFlex.set(-0.5);
                break;
        }
    }

    @Override
    public void stopIntake() {
        _intakeControlSparkFlex.set(0);

    }

    public void setIntakeFeedState(Hopper.IntakeFeedState intakeFeedState) {
        switch (intakeFeedState) {
            case INWARDS:
                _intakeFeedSparkFlex.set(.5);
                break;
            case OUTWARDS:
                _intakeFeedSparkFlex.set(-.5);
                break;
            case STOPPED:
            default:
                stopIntake();
                break;
        }
    }

}
