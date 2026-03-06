package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _intakeSolenoid;
    private SparkFlex _upperFeedSparkFlex;
    private SparkFlex _lowerFeedSparkFlex;
    private SparkFlex _intakeFeedSparkFlex;

    public HopperReal() {
        _intakeSolenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.IntakeForwardChannel,
                HopperMap.IntakeReverseChannel);

        SparkFlexConfig hopperFeedConfig = new SparkFlexConfig();
        // hopperFeedConfig.smartCurrentLimit(60, 50);
        hopperFeedConfig.smartCurrentLimit(10, 5);
        _upperFeedSparkFlex = new SparkFlex(HopperMap.UPPER_FEED_CAN_ID, MotorType.kBrushless);
        _upperFeedSparkFlex.configure(hopperFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _lowerFeedSparkFlex = new SparkFlex(HopperMap.LOWER_FEED_CAN_ID, MotorType.kBrushless);
        _lowerFeedSparkFlex.configure(hopperFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _intakeFeedSparkFlex = new SparkFlex(HopperMap.INTAKE_CAN_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(60, 50);
        _intakeFeedSparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {

    }

    @Override
    public void setFeedSpeed(double speed) {
        _upperFeedSparkFlex.set(-speed);
        _lowerFeedSparkFlex.set(speed);
    }

    @Override
    public void feedStop() {
        _upperFeedSparkFlex.stopMotor();
        _lowerFeedSparkFlex.stopMotor();
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
