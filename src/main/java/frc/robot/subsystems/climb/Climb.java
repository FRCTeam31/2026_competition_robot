package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;

import org.prime.util.SubsystemMechanism;

public class Climb extends SubsystemBase {
    private IClimb _climb;
    private SubsystemMechanism _mechanism = new SubsystemMechanism(
            () -> new Pose3d(SuperStructure.Swerve.EstimatedRobotPose)
    );

    public enum ClimbState {
        UP,
        STOPPED,
        DOWN
    }

    public enum SupportState {
        RAISED,
        LOWERED
    }

    public enum FrictionBrakeState {
        APPLIED,
        RELEASED
    }

    /**
     * Represents the current state in the
     * climbing process used to restrict climb
     * commands from running out of order or
     * at the same time
     */
    public enum ClimbControlState {
        RESET,
        RESETTING,
        SETUP_IN_PROGRESS,
        SETUP_DONE,
        CLIMBING_UP,
        HAS_CLIMBED,
        CLIMBING_DOWN,
        CLIMBING_DONE
    }

    public Climb(boolean isReal) {
        setName("Climb");
        _climb = isReal ? new ClimbReal() : new ClimbSim();

        initMechanism();
    }

    private void initMechanism() {
        _mechanism.createMechanism(2,3);
        _mechanism.createRoot("ClimbRoot", 0, 0);

        _mechanism.appendLigament(
                "Climb",
                0.8,
                0,
                () -> {
                    if (SuperStructure.Climb.climbState == ClimbState.UP) {
                        return new Translation3d(0, 0, 0.5);
                    } else {
                        return Translation3d.kZero;
                    }
                },
                () -> Rotation3d.kZero
        );
    }

    private void actOnState(ClimbInputsAutoLogged inputs) {
        switch (inputs.climbState) {
            case UP:
                _climb.controlClimb(inputs.upperLimitSwitch ? 0 : 0.5);
                break;
            case DOWN:
                _climb.controlClimb(inputs.lowerLimitSwitch ? 0 : -0.5);
                break;
            case STOPPED:
            default:
                _climb.controlClimb(0);
                break;
        }

        _climb.controlSupport(inputs.supportState == SupportState.RAISED
                ? DoubleSolenoid.Value.kReverse
                : DoubleSolenoid.Value.kForward);

        _climb.controlFrictionBrake(inputs.frictionBrakeState == FrictionBrakeState.APPLIED
                ? DoubleSolenoid.Value.kForward
                : DoubleSolenoid.Value.kReverse);

    }

    @Override
    public void periodic() {
        _mechanism.updateMechanism();
        Logger.recordOutput("Climb/ClimbMechanism", _mechanism.getMechanism());
        Logger.recordOutput("Climb/ClimbMechanismPose", _mechanism.getFieldRelativePose("Climb"));
        Logger.recordOutput("Climb/ClimbMechanismPoseGenerated", _mechanism.getMechanism().generate3dMechanism().getFirst());

        _climb.updateInputs(SuperStructure.Climb);
        Logger.processInputs(getName(), SuperStructure.Climb);

        actOnState(SuperStructure.Climb);
    }

    // #region Commands

    /**
     * Sets the climb motor state
     * @param state The desired climb state (UP, DOWN, STOPPED)
     * @return Command to set the state
     */
    public Command setClimb(ClimbState state) {
        return this.runOnce(() -> SuperStructure.Climb.climbState = state);
    }

    /**
     * Sets the support solenoid state
     * @param state The desired support state (RAISED, LOWERED)
     * @return Command to set the state
     */
    public Command setSupport(SupportState state) {
        return this.runOnce(() -> SuperStructure.Climb.supportState = state);
    }

    /**
     * Sets the friction brake state
     * @param state The desired brake state (APPLIED, RELEASED)
     * @return Command to set the state
     */
    public Command setBrake(FrictionBrakeState state) {
        return this.runOnce(() -> SuperStructure.Climb.frictionBrakeState = state);
    }

    // #endregion
}
