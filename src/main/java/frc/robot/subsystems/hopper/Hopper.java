package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;

public class Hopper extends SubsystemBase {
    private IHopper _hopper;

    public Hopper(boolean isReal) {
        _hopper = isReal ? new HopperReal() : new HopperSim();
    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);
    }
}
