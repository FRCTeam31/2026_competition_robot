package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.climb.ClimbMap;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SuperStructure;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Hopper extends SubsystemBase {
    private IHopper _hopper;

    // Hopper Mechanism
    private LoggedMechanism2d _hopperMechanism;
    private LoggedMechanismRoot2d _hopperRoot;
    private LoggedMechanismLigament2d _hopperExtensionLigament;
    private LoggedMechanismLigament2d _hopperStaticLigament;

    // Intake Mechanism
    private LoggedMechanism2d _intakeMechanism;
    private LoggedMechanismRoot2d _intakeRoot;
    private LoggedMechanismLigament2d _intakeLigament;

    @SuppressWarnings("unused")
    private Trigger _pulseHopperTrigger = new Trigger(
            () -> SuperStructure.Hopper.ExtensionState == ExtensionState.PULSING)
            .whileTrue(pulseHopperPrivateCommand());

    public enum ExtensionState { // For extending and retracting Hopper
        IN,
        OUT,
        OFF,
        PULSING
    }

    public enum TransferFeedState { // For feeding in and out to the shooter
        INWARDS,
        OUTWARDS,
        STOPPED
    }

    public enum IntakeFeedState { // For intake wheels
        INWARDS,
        OUTWARDS,
        STOPPED
    }

    public enum IntakeControlState { // For intake rotation
        IN,
        OUT,
        OFF
    }

    public Hopper(boolean isReal) {
        setName("Hopper");

        _hopper = isReal ? new HopperReal() : new HopperSim();

        initMechanisms();
    }

    private void initMechanisms() {
        _hopperMechanism = new LoggedMechanism2d(2, 2);
        _hopperRoot = _hopperMechanism.getRoot("HopperRoot", 0, 0);
        _hopperExtensionLigament = new LoggedMechanismLigament2d("HopperExtensionLigament", 0, 0);
        _hopperRoot.append(_hopperExtensionLigament);
        _hopperStaticLigament = new LoggedMechanismLigament2d("HopperStaticLigament", 0.415, 0);
        _hopperExtensionLigament.append(_hopperStaticLigament);

        _intakeMechanism = new LoggedMechanism2d(2, 2);
        _intakeRoot = _intakeMechanism.getRoot("IntakeRoot", 0, 0);
        _intakeLigament = new LoggedMechanismLigament2d("IntakeLigament", 0.319, 145);
        _intakeRoot.append(_intakeLigament);
    }

    private void actOnState(HopperInputsAutoLogged inputs) {
        // Feed motor control
        switch (inputs.TransferFeedState) {
            case INWARDS:
                _hopper.setFeedSpeed(0.5);
                break;
            case OUTWARDS:
                _hopper.setFeedSpeed(-0.5);
                break;
            case STOPPED:
            default:
                _hopper.setFeedSpeed(0);
                break;
        }

        // Intake solenoid control
        switch (inputs.IntakeControlState) {
            case IN:
            default:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kReverse);
                break;
            case OUT:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kForward);
                break;
            case OFF:
                _hopper.setIntakePosition(DoubleSolenoid.Value.kOff);
        }

        // Intake feed motor control
        switch (inputs.IntakeFeedState) {
            case INWARDS:
                _hopper.setIntakeFeedSpeed(.5);
                break;
            case OUTWARDS:
                _hopper.setIntakeFeedSpeed(-.5);
                break;
            case STOPPED:
            default:
                _hopper.stopIntake();
                break;
        }

        // Hopper solenoid control
        switch (inputs.ExtensionState) {
            case IN:
            default:
                _hopper.setHopper(DoubleSolenoid.Value.kReverse);
                break;
            case OUT:
                _hopper.setHopper(DoubleSolenoid.Value.kForward);
                break;
            case OFF:
                _hopper.setHopper(DoubleSolenoid.Value.kOff);
                break;
        }
    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);

        _hopperExtensionLigament.setLength(
                SuperStructure.Hopper.ExtensionState == ExtensionState.OUT
                        ? 0.281
                        : 0
        );
        _intakeLigament.setAngle(
                SuperStructure.Hopper.IntakeControlState == IntakeControlState.OUT
                        ? 0
                        : 145
        );

        Pose3d hopperComponent = _hopperMechanism.generate3dMechanism().get(1);
        Pose3d intakeComponent = _intakeMechanism.generate3dMechanism().get(0);

        Pose3d robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);
        Translation3d robotTranslation = robotPose.getTranslation();
        Rotation3d robotRotation = robotPose.getRotation();

        SuperStructure.Hopper.hopperComponentPose = new Pose3d(
                hopperComponent.getTranslation().plus(HopperMap.HOPPER_ROOT_POSITION).plus(robotTranslation),
                hopperComponent.getRotation()
        ).rotateAround(robotTranslation, robotRotation);
        SuperStructure.Hopper.intakeComponentPose = new Pose3d(
                intakeComponent.getTranslation().plus(HopperMap.INTAKE_ROOT_POSITION).plus(robotTranslation),
                intakeComponent.getRotation()
        ).rotateAround(robotTranslation, robotRotation);
        SuperStructure.Hopper.intakeFeedComponentPose = robotPose
                .plus(new Transform3d(
                        HopperMap.INTAKE_ROOT_POSITION,
                        Rotation3d.kZero
                ))
                .plus(new Transform3d(
                        intakeComponent.getTranslation(),
                        intakeComponent.getRotation()
                )).plus(new Transform3d(
                        HopperMap.INTAKE_FEED_ROOT_POSITION,
                        new Rotation3d(
                                SuperStructure.Hopper.hopperFeedPosition * Math.PI * 2,
                                0,
                                Math.PI / 2
                        )
                ));
        SuperStructure.Hopper.topFeedBarComponentPose = robotPose.plus(new Transform3d(
                HopperMap.TOP_FEED_BAR_ROOT_POSITION,
                new Rotation3d(
                        SuperStructure.Hopper.intakeFeedPosition * Math.PI * 2,
                        0,
                        Math.PI / 2
                )
        ));
        SuperStructure.Hopper.bottomFeedBarComponentPose = robotPose.plus(new Transform3d(
                HopperMap.BOTTOM_FEED_BAR_ROOT_POSITION,
                new Rotation3d(
                        -SuperStructure.Hopper.intakeFeedPosition * Math.PI * 2,
                        0,
                        Math.PI / 2
                )
        ));

        Logger.recordOutput("Hopper/HopperMechanism", _hopperMechanism);
        Logger.recordOutput("Hopper/IntakeMechanism", _intakeMechanism);

        Logger.processInputs(getName(), SuperStructure.Hopper);

        actOnState(SuperStructure.Hopper);
    }

    // #region Commands

    private Command pulseHopperPrivateCommand() {
        return this.run(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.IN)
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
                .andThen(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.OUT)
                .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay));
    }

    /**
     * Sets the hopper solenoid state
     * @param state The desired hopper state (IN, OUT, OFF, PULSING)
     * @return Command to set the state
     */
    public Command setHopper(ExtensionState state) {
        return this.runOnce(() -> SuperStructure.Hopper.ExtensionState = state);
    }

    /**
     * Sets the transfer feed motor state
     * @param state The desired feed state (INWARDS, OUTWARDS, STOPPED)
     * @return Command to set the state
     */
    public Command setFeed(TransferFeedState state) {
        return this.runOnce(() -> SuperStructure.Hopper.TransferFeedState = state);
    }

    /**
     * Sets the intake feed motor state
     * @param state The desired intake feed state (INWARDS, OUTWARDS, STOPPED)
     * @return Command to set the state
     */
    public Command setIntakeFeed(IntakeFeedState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeFeedState = state);
    }

    /**
     * Sets the intake solenoid control state
     * @param state The desired intake control state (IN, OUT, OFF)
     * @return Command to set the state
     */
    public Command setIntakeControl(IntakeControlState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = state);
    }

    // #endregion
}
