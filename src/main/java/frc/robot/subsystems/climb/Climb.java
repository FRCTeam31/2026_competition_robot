package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Climb extends SubsystemBase {
    private IClimb _climb;

    // Climb Mechanism
    private LoggedMechanism2d _climbMechanism;
    private LoggedMechanismRoot2d _climbRoot;
    private LoggedMechanismLigament2d _climbExtensionLigament;
    private LoggedMechanismLigament2d _climbStaticLigament;

    // Support Mechanism
    private LoggedMechanism2d _supportMechanism;
    private LoggedMechanismRoot2d _supportRoot;
    private LoggedMechanismLigament2d _supportLigament;

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

        initMechanisms();
    }

    private void initMechanisms() {
        _climbMechanism = new LoggedMechanism2d(3, 3);
        _climbRoot = _climbMechanism.getRoot("ClimbRoot", 0, 0);
        _climbExtensionLigament = new LoggedMechanismLigament2d("ClimbExtensionLigament", 0, 90, 9, new Color8Bit(Color.kDarkGray));
        _climbRoot.append(_climbExtensionLigament);
        _climbStaticLigament = new LoggedMechanismLigament2d("ClimbStaticLigament", 0.4191, 0, 8, new Color8Bit(Color.kLightBlue));
        _climbExtensionLigament.append(_climbStaticLigament);

        _supportMechanism = new LoggedMechanism2d(1,1);
        _supportRoot = _supportMechanism.getRoot("SupportRoot", 0, 0);
        _supportLigament = new LoggedMechanismLigament2d("SupportLigament", 0.2286, 90, 5, new Color8Bit(Color.kDarkBlue));
        _supportRoot.append(_supportLigament);
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
        _climb.updateInputs(SuperStructure.Climb);

        _climbExtensionLigament.setLength(SuperStructure.Climb.climbExtension);
        _supportLigament.setAngle(SuperStructure.Climb.supportAngle);

        Pose3d climbComponent = _climbMechanism.generate3dMechanism().get(1);
        Pose3d supportComponent = _supportMechanism.generate3dMechanism().get(0);

        Pose3d robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);
        Translation3d robotTranslation = robotPose.getTranslation();
        Rotation3d robotRotation = robotPose.getRotation();

        SuperStructure.Climb.climbComponentPose = new Pose3d(
                climbComponent.getTranslation().plus(ClimbMap.CLIMB_ROOT_POSITION).plus(robotTranslation),
                climbComponent.getRotation()
        ).rotateAround(robotTranslation, robotRotation);
        SuperStructure.Climb.supportComponentPose = new Pose3d(
                supportComponent.getTranslation().plus(ClimbMap.SUPPORT_ROOT_POSITION).plus(robotTranslation),
                supportComponent.getRotation()
        ).rotateAround(robotTranslation, robotRotation);

        Logger.recordOutput("Climb/ClimbMechanism", _climbMechanism);
        Logger.recordOutput("Climb/SupportMechanism", _supportMechanism);

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
