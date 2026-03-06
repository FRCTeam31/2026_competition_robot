package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretV3 {
    private SparkFlex _flywheelLeft;
    private SparkFlex _flywheelRight;
    private SparkFlex _feed;

    public TurretV3() {
        _flywheelLeft = new SparkFlex(19, MotorType.kBrushless);
        _flywheelRight = new SparkFlex(20, MotorType.kBrushless);
        _feed = new SparkFlex(16, MotorType.kBrushless);
    }

    public void setShooter(double speed) {
        _flywheelLeft.set(speed);
        _flywheelRight.set(-speed);
    }

    public void stopShooter() {
        _flywheelLeft.stopMotor();
        _flywheelRight.stopMotor();
    }

    public void setFeed(double speed) {
        _feed.set(speed);
    }

    public void stopFeed() {
        _feed.stopMotor();
    }

    public Command setShooterCommand(double speed) {
        return Commands.runOnce(() -> setShooter(speed));
    }

    public Command stopShooterCommand() {
        return Commands.runOnce(() -> stopShooter());
    }

    public Command setFeedCommand(double speed) {
        return Commands.runOnce(() -> setFeed(speed));
    }

    public Command stopFeedCommand() {
        return Commands.runOnce(() -> stopFeed());
    }
}
