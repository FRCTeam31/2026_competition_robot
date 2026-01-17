package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.hopper.Hopper.FeedState;
import frc.robot.subsystems.hopper.Hopper.IntakeControlState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;

public class HopperSim implements IHopper {

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setHopper(Value value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHopper'");
    }

    @Override
    public void toggleHopper() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toggleHopper'");
    }

    @Override
    public void setFeedSpeed(FeedState feedState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFeedSpeed'");
    }

    @Override
    public void feedStop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'feedStop'");
    }

    @Override
    public void stopIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopIntake'");
    }

    @Override
    public void setIntakePosition(IntakeControlState controlState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIntakePosition'");
    }

    @Override
    public void setIntakeFeedState(IntakeFeedState intakeFeedState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIntakeFeedState'");
    }

}
