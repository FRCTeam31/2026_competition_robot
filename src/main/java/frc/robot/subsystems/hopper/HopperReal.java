package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.Container;

public class HopperReal implements IHopper {
    private DoubleSolenoid _solenoid;

    public HopperReal() {
        _solenoid = new DoubleSolenoid(Container.Pneumatics.getPneumaticsControlModuleId(),
                Container.Pneumatics.getPneumaticsControlModuleType(), HopperMap.ForwardChannel,
                HopperMap.ReverseChannel);
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
}
