package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure;

public class Turret extends SubsystemBase {
    private ITurret _turret;

    public enum FlywheelStates {
        IDLE,
        SHOOT_MANUAL,
        SHOOT_AUTO,
        PASSING,
        STOPPED
    }

    public enum TargetingStates {
        MANUAL_CONTROL,
        AUTO_ASSISTED
    }

    public Turret(boolean isReal) {
        _turret = isReal ? new TurretReal() : new TurretSim();
    }

    @Override
    public void periodic() {
        _turret.updateInputs(SuperStructure.Turret);
    }
}
