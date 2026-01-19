package frc.robot.subsystems.climb;

public class Climb {
    private IClimb _climb;

    public enum ClimbState {
        // TODO: Determine climber states
    }

    public enum SupportState {
        // TODO: Determine support states
    }

    public Climb(boolean isReal) {
        _climb = isReal ? new ClimbReal() : new ClimbSim();
    }
}
