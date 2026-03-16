package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.prime.dashboard.DashboardSection;

public class DriverDashboard extends DashboardSection {
        private final String _fieldName = "Field";
        private final Field2d _fieldWidget;

        // Widgets
        private final HashMap<String, BooleanEvent> _booleanEvents = new HashMap<>();

        public DriverDashboard() {
                super("Driver");

                // Add complex data like the field view
                _fieldWidget = new Field2d();
                putData(_fieldName, _fieldWidget);
        }

        public void setFieldRobotPose(Pose2d pose) {
                _fieldWidget.setRobotPose(pose);
        }

        public FieldObject2d getFieldTargetPose() {
                return _fieldWidget.getObject("target pose");
        }

        public void setFieldTargetPose(Pose2d pose) {
                getFieldTargetPose().setPose(pose);
        }

        public FieldObject2d getFieldVisionEstimationPose() {
                return _fieldWidget.getObject("VisionEstimation");
        }

        public void setFieldVisionEstimationPose(Pose2d pose) {
                getFieldVisionEstimationPose().setPose(pose);
        }

        public FieldObject2d getFieldPath() {
                return _fieldWidget.getObject("path");
        }

        public void setFieldPath(List<Pose2d> poses) {
                getFieldPath().setPoses(poses);
        }

        public void clearFieldPath() {
                getFieldPath().setPoses(new ArrayList<>());
        }

        public BooleanEvent getSwitchBooleanEvent(String name, boolean defaultValue) {
                if (!_booleanEvents.containsKey(name)) {
                        putBoolean(name, defaultValue);
                        _booleanEvents.put(name, new BooleanEvent(Robot.EventLoop, () -> getBoolean(name, defaultValue))
                                        .debounce(0.2));
                }

                return _booleanEvents.get(name);
        }
}
