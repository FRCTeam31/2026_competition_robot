package frc.robot.subsystems.hopper;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _hopperSolenoid;
    private DoubleSolenoid _intakeSolenoid;
    private SparkFlex _feedSparkFlex;
    private SparkFlex _intakeFeedSparkFlex;


    public HopperReal() {
        _hopperSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.HopperForwardChannel,
                HopperMap.HopperReverseChannel);
        _intakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.IntakeForwardChannel,
                HopperMap.IntakeReverseChannel);
        _feedSparkFlex = new SparkFlex(HopperMap.CANID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {

    }

    @Override
    public void setHopper(DoubleSolenoid.Value value) {
        _hopperSolenoid.set(value);
    }

    @Override
    public void toggleHopper() {
        _hopperSolenoid.toggle();
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
                _intakeSolenoid.set(DoubleSolenoid.Value.kForward);
                break;
            case IN:
            default:
                _intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
        }
    }

    @Override
    public void stopIntake() {
        _intakeSolenoid.set(DoubleSolenoid.Value.kOff);
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
