package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;

@SuppressWarnings("unused")
public class HopperSim implements IHopper {

    private DoubleSolenoid.Value _hopperState = DoubleSolenoid.Value.kOff;
    private DoubleSolenoid.Value _intakeState = DoubleSolenoid.Value.kOff;

    private double _feedRollersSpeed = 0;
    private double _intakeFeedSpeed = 0;

    public HopperSim() {
    }

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {
        // Mirror any logged inputs here if fields are added to HopperInputsAutoLogged
    }

    @Override
    public void setHopper(DoubleSolenoid.Value value) {
        _hopperState = value;
    }

    @Override
    public void setFeedSpeed(double speed) {
        _feedRollersSpeed = speed;
    }

    @Override
    public void feedStop() {
        _feedRollersSpeed = 0;
    }

    @Override
    public void setIntakePosition(DoubleSolenoid.Value value) {
        _intakeState = value;
    }

    @Override
    public void stopIntake() {
        _intakeState = DoubleSolenoid.Value.kOff;
    }

    @Override
    public void setIntakeFeedSpeed(double speed) {
        _intakeFeedSpeed = speed;
    }
}