package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroMapleSim implements IGyro {
    private final GyroSimulation _gyro;

    public GyroMapleSim(GyroSimulation gyro) {
        _gyro = gyro;
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs, double omegaRadiansPerSecond) {
        // discretize rotational speed
        var delta = omegaRadiansPerSecond * 0.02;
        var currentAngle = _gyro.getGyroReading();
        var newGyroAngle = Rotation2d.fromRadians(currentAngle.getRadians() + delta);

        _gyro.setRotation(newGyroAngle);

        inputs.Rotation = _gyro.getGyroReading();
        inputs.AccelerationX = 0;
        inputs.AccelerationY = 0;
        inputs.AccelerationZ = 0;
    }

    @Override
    public void reset() {
        _gyro.setRotation(Rotation2d.kZero);
    }

    @Override
    public void reset(double angle) {
        _gyro.setRotation(Rotation2d.fromDegrees(angle));
    }
}
