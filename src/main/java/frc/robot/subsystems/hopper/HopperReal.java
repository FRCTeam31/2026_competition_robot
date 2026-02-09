package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _hopperSolenoid;
    private DoubleSolenoid _intakeSolenoid;
    private SparkFlex _feedRollersSparkFlex;
    private SparkFlex _intakeFeedSparkFlex;

    public HopperReal() {
        _hopperSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.HopperForwardChannel,
                HopperMap.HopperReverseChannel);
        _intakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.IntakeForwardChannel,
                HopperMap.IntakeReverseChannel);
        _feedRollersSparkFlex = new SparkFlex(HopperMap.CANID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {
        inputs.intakeFeedPosition = _intakeFeedSparkFlex.getEncoder().getPosition();
        inputs.hopperFeedPosition = _feedRollersSparkFlex.getEncoder().getPosition() / HopperMap.HOPPER_FEED_GEAR_RATIO;
    }

    @Override
    public void setHopper(DoubleSolenoid.Value value) {
        _hopperSolenoid.set(value);
    }

    @Override
    public void setFeedSpeed(double speed) {
        _feedRollersSparkFlex.set(speed);
    }

    @Override
    public void feedStop() {
        _feedRollersSparkFlex.stopMotor();
    }

    @Override
    public void setIntakePosition(DoubleSolenoid.Value value) {
        _intakeSolenoid.set(value);
    }

    @Override
    public void stopIntake() {
        _intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void setIntakeFeedSpeed(double speed) {
        _intakeFeedSparkFlex.set(speed);
    }

}
