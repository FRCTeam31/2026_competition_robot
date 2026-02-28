package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _intakeSolenoid;
    private SparkFlex _feedRollersSparkFlex;
    private SparkFlex _intakeFeedSparkFlex;

    public HopperReal() {
        _intakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.IntakeForwardChannel,
                HopperMap.IntakeReverseChannel);
        _feedRollersSparkFlex = new SparkFlex(HopperMap.FEED_CAN_ID, MotorType.kBrushless);
        _intakeFeedSparkFlex = new SparkFlex(HopperMap.INTAKE_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {

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
