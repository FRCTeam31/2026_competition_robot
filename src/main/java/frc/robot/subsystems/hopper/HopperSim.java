package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.hopper.Hopper.FeedState;
import frc.robot.subsystems.hopper.Hopper.IntakeControlState;
import frc.robot.subsystems.hopper.Hopper.IntakeFeedState;

public class HopperSim implements IHopper {

    // Simulated state variables
    private Value hopperSolenoidState = Value.kOff;
    private FeedState currentFeedState = FeedState.STOPPED;
    private IntakeControlState currentIntakePosition = IntakeControlState.IN;
    private IntakeFeedState currentIntakeFeedState = IntakeFeedState.STOPPED;

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
    public void setFeedSpeed(FeedState feedState) {
        currentFeedState = feedState;
    }

    @Override
    public void feedStop() {
        currentFeedState = FeedState.STOPPED;
    }

    @Override
    public void stopIntake() {
        currentIntakePosition = IntakeControlState.IN;
    }

    @Override
    public void setIntakePosition(IntakeControlState controlState) {
        currentIntakePosition = controlState;
    }

    @Override
    public void setIntakeFeedState(IntakeFeedState intakeFeedState) {
        currentIntakeFeedState = intakeFeedState;
    }

    /**
     * Simulates the intake position movement based on the current control state
     */
    private void updateIntakePosition() {
        switch (currentIntakePosition) {
            case OUT:
                if (!outLimitSwitch) {
                    intakePosition += INTAKE_MOVE_SPEED;
                    if (intakePosition > 1.0) {
                        intakePosition = 1.0;
                    }
                }
                break;
            case IN:
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