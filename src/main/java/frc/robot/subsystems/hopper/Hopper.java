package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;

public class Hopper extends SubsystemBase {
    private IHopper _hopper;
    private DoubleSolenoid.Value _lastHopperValue;

    public Hopper(boolean isReal) {
        _hopper = isReal ? new HopperReal() : new HopperSim();
    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);
    }

    public Command setHopperOut() {
        return this.runOnce(() -> {
            _hopper.setHopper(DoubleSolenoid.Value.kForward);
            _lastHopperValue = DoubleSolenoid.Value.kForward;
        });
    }

    public Command setHopperIn() {
        return this.runOnce(() -> {
            _hopper.setHopper(DoubleSolenoid.Value.kReverse);
            _lastHopperValue = DoubleSolenoid.Value.kReverse;
        });
    }

    public Command pulseHopper() {
        return this.run(() -> _hopper.toggleHopper())
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
                .finallyDo(() -> _hopper.setHopper(_lastHopperValue));
    }
}
