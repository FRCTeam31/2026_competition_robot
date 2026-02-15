package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConfig;
import frc.robot.subsystems.climb.ClimbMap;
import org.littletonrobotics.junction.Logger;
import org.prime.subsystems.LoggedSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SuperStructure;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Hopper extends LoggedSubsystem {
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

    public enum HopperIntakeState { // For intake rotation
        IN,
        OUT,
        OFF
    }

    public Hopper() {
        setName("Hopper");

        _hopper = Robot.isReal()
                ? new HopperReal()
                : new HopperSim();

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

        // Intake and Hopper solenoid control
        switch (inputs.IntakeControlState) {
            case IN:
            default:
                _hopper.setHopper(DoubleSolenoid.Value.kReverse);
                _hopper.setIntakePosition(DoubleSolenoid.Value.kReverse);
                break;
            case OUT:
                _hopper.setHopper(DoubleSolenoid.Value.kForward);
                _hopper.setIntakePosition(DoubleSolenoid.Value.kForward);
                break;
            case OFF:
                _hopper.setHopper(DoubleSolenoid.Value.kOff);
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

    }

    @Override
    public void periodic() {
        _hopper.updateInputs(SuperStructure.Hopper);

        // Updates the hopper ligaments
        if (SuperStructure.Hopper.IntakeControlState == HopperIntakeState.OUT) {
            _hopperExtensionLigament.setLength(0.281);
            _intakeLigament.setAngle(0);
        } else {
            _hopperExtensionLigament.setLength(0);
            _intakeLigament.setAngle(145);
        }


        // Gets the specific generated poses that represent the hopper components
        Pose3d hopperComponent = _hopperMechanism.generate3dMechanism().get(1);
        Pose3d intakeComponent = _intakeMechanism.generate3dMechanism().get(0);

        Pose3d robotPose = new Pose3d(SuperStructure.Swerve.EstimatedRobotPose);
        Translation3d robotTranslation = robotPose.getTranslation();
        Rotation3d robotRotation = robotPose.getRotation();

        // Convert the generated robot relative poses to field relative
        SuperStructure.Hopper.hopperComponentPose = new Pose3d(
                hopperComponent.getTranslation().plus(HopperMap.HOPPER_ROOT_POSITION).plus(robotTranslation),
                hopperComponent.getRotation()
        ).rotateAround(robotTranslation, robotRotation);
        SuperStructure.Hopper.intakeComponentPose = new Pose3d(
                intakeComponent.getTranslation().plus(HopperMap.INTAKE_ROOT_POSITION).plus(robotTranslation),
                intakeComponent.getRotation()
        ).rotateAround(robotTranslation, robotRotation);

        // Creates poses representing the rotation of the intake and hopper feeds, used only in advanced robot
        // visualization
        if (RobotConfig.USE_ADVANCED_ROBOT_VISUALIZATION) {
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
        }

        // Log the Mechanism2ds using AdvantageKit's Logger class
        Logger.recordOutput(getName() + "/HopperMechanism", _hopperMechanism);
        Logger.recordOutput(getName() + "/IntakeMechanism", _intakeMechanism);

        processInputs(SuperStructure.Hopper);

        actOnState(SuperStructure.Hopper);
    }

    // #region Commands

    // private Command pulseHopperPrivateCommand() {
    //     return this.run(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.IN)
    //             .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay))
    //             .andThen(() -> SuperStructure.Hopper.ExtensionState = ExtensionState.OUT)
    //             .andThen(Commands.waitSeconds(HopperMap.HopperPulseDelay));
    // }

    /**
     * Sets the hopper solenoid state
     * @param state The desired hopper state (IN, OUT, OFF, PULSING)
     * @return Command to set the state
     */

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
    public Command setHopperIntakeControl(HopperIntakeState state) {
        return this.runOnce(() -> SuperStructure.Hopper.IntakeControlState = state);
    }

    // #endregion
}
