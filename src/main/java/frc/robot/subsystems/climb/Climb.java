package frc.robot.subsystems.climb;

public class Climb {
    private IClimb _climb;

    public enum ClimbState {
        Up,
        Stopped,
        Down
    }

    public enum SupportState {
        // TODO: Determine support states
    }

    public enum FrictionBrakeState {
        Applied,
        Released
    }

    public Climb(boolean isReal) {
        _climb = isReal ? new ClimbReal() : new ClimbSim();
    }
}
