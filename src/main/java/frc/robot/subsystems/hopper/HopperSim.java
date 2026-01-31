package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.hopper.Hopper.FeedState;
import frc.robot.subsystems.hopper.Hopper.IntakeControlState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;

public class HopperSim implements IHopper {

    // Simulated state variables
    private Value hopperSolenoidState = Value.kOff;
    private double currentFeedSpeed = 0;
    private DoubleSolenoid.Value currentIntakePosition = DoubleSolenoid.Value.kOff;
    private double currentIntakeFeedSpeed = 0;

    // Simulated intake position (0.0 = fully in, 1.0 = fully out)
    private double intakePosition = 0.0;
    private static final double INTAKE_MOVE_SPEED = 0.02; // Position change per update cycle

    // Simulated limit switches
    private boolean inLimitSwitch = true; // Start at IN position
    private boolean outLimitSwitch = false;

    @Override
    public void updateInputs(HopperInputsAutoLogged inputs) {
        // Update simulated intake position based on control state
        updateIntakePosition();

        // Update limit switches based on position
        inLimitSwitch = (intakePosition <= 0.0);
        outLimitSwitch = (intakePosition >= 1.0);

        // Update inputs
        inputs.intakeINLimitSwitch = inLimitSwitch;
        inputs.intakeOUTLimitSwitch = outLimitSwitch;
    }

    @Override
    public void setHopper(Value value) {
        hopperSolenoidState = value;
    }

    @Override
    public void toggleHopper() {
        if (hopperSolenoidState == Value.kForward) {
            hopperSolenoidState = Value.kReverse;
        } else if (hopperSolenoidState == Value.kReverse) {
            hopperSolenoidState = Value.kForward;
        } else {
            // If kOff, default to kForward
            hopperSolenoidState = Value.kForward;
        }
    }

    @Override
    public void setFeedSpeed(double speed) {
        currentFeedSpeed = speed;
    }

    @Override
    public void feedStop() {
        currentFeedSpeed = 0;
    }

    @Override
    public void stopIntake() {
        currentIntakePosition = Value.kReverse;
    }

    @Override
    public void setIntakePosition(DoubleSolenoid.Value value) {
        currentIntakePosition = value;
    }

    @Override
    public void setIntakeFeedSpeed(double speed) {
        currentIntakeFeedSpeed = speed;
    }

    /**
     * Simulates the intake position movement based on the current control state
     */
    private void updateIntakePosition() {
        switch (currentIntakePosition) {
            case kForward:
                if (!outLimitSwitch) {
                    intakePosition += INTAKE_MOVE_SPEED;
                    if (intakePosition > 1.0) {
                        intakePosition = 1.0;
                    }
                }
                break;
            case kReverse:
            default:
                if (!inLimitSwitch) {
                    intakePosition -= INTAKE_MOVE_SPEED;
                    if (intakePosition < 0.0) {
                        intakePosition = 0.0;
                    }
                }
                break;
        }
    }
}