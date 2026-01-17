package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _solenoid;
    private SparkFlex _sparkFlex;

    public HopperReal() {
        _solenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.ForwardChannel,
                HopperMap.ReverseChannel);
        _sparkFlex = new SparkFlex(HopperMap.CANID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {

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
        // if (feedState == Hopper.FeedState.INWARDS) {
        //     _sparkFlex.set(0.5);
        // }
        // if (feedState == Hopper.FeedState.OUTWARDS) {
        //     _sparkFlex.set(-0.5);
        // }
        // if (feedState == Hopper.FeedState.STOPPED) {}
        switch (feedState) {
            case INWARDS:
                _sparkFlex.set(0.5);
                break;
            case OUTWARDS:
                _sparkFlex.set(-0.5);
                break;
            case STOPPED:
            default:
                _sparkFlex.set(0);
                break;
        }
    }

    @Override
    public void feedStop() {
        _sparkFlex.stopMotor();
    }
}
