package frc.robot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Static class containing field target positions and helper methods. Coordinates derived from Pathplanner field model.
 * Field origin is bottom-left corner with Blue Alliance on the left side when viewed from above.
 * Based on welded field dimensions, not Andymark field.
 * https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
 */
public class FieldTargets {
        public enum TargetType {
                kHub,
                kPassing,
                kFarPassing,
                kDead,
                kNone
        }

        public static final AprilTagFieldLayout FieldTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.kDefaultField);

        public static final double FIELD_TOP = Units.inchesToMeters(317.69);
        public static final double FIELD_Y_MIDDLE = FIELD_TOP / 2;
        public static final double FIELD_BOTTOM = 0;
        public static final double FIELD_LEFT = 0;
        public static final double FIELD_RIGHT = Units.inchesToMeters(651.22);
        public static final double HUB_HEIGHT = Units.inchesToMeters(72);
        public static final double HUB_SCORING_HEIGHT = Units.inchesToMeters(68);
        public static final double ALLIANCE_ZONE_WIDTH = Units.inchesToMeters(156.61);

        // Hub Targets
        private static final List<Double> Usable_Center_Hub_Targets = List.of(
                        18d,
                        26d,
                        21d,
                        5d,
                        2d,
                        10d);

        private static final List<Double> Usable_Left_Hub_Targets = List.of(
                        9d,
                        11d,
                        25d,
                        27d);

        private static final List<Double> Usable_Right_Hub_Targets = List.of(
                        24d,
                        8d);

        private static final Transform3d Center_Hub_Target_Offset = new Transform3d(Units.inchesToMeters(-23.5), 0, 0,
                        Rotation3d.kZero);
        private static final Transform3d Left_Hub_Target_Offset = Center_Hub_Target_Offset
                        .plus(new Transform3d(0, Units.inchesToMeters(14), 0, Rotation3d.kZero));
        private static final Transform3d Right_Hub_Target_Offset = Center_Hub_Target_Offset
                        .plus(new Transform3d(0, Units.inchesToMeters(-14), 0, Rotation3d.kZero));

        private static final Pose3d Blue_Hub_Position = new Pose3d(
                        new Translation3d(
                                        Units.inchesToMeters(182.11),
                                        FIELD_Y_MIDDLE,
                                        HUB_HEIGHT),
                        Rotation3d.kZero);
        private static final Pose3d Red_Hub_Position = new Pose3d(
                        new Translation3d(
                                        Units.inchesToMeters(469.11),
                                        FIELD_Y_MIDDLE,
                                        HUB_HEIGHT),
                        Rotation3d.kZero);

        public static Pose3d GetCurrentAllianceHubPosition() {
                return Robot.onBlueAlliance()
                                ? Blue_Hub_Position
                                : Red_Hub_Position;
        }

        // Passing positions
        private static final Pose3d Blue_North_Passing_Position = new Pose3d(
                        new Translation3d(4.337, 5.553, 0),
                        Rotation3d.kZero);
        private static final Pose3d Blue_South_Passing_Position = new Pose3d(
                        new Translation3d(4.337, 2.527, 0),
                        Rotation3d.kZero);
        private static final Pose3d Red_North_Passing_Position = new Pose3d(
                        new Translation3d(12.193, 5.553, 0),
                        Rotation3d.kZero);
        private static final Pose3d Red_South_Passing_Position = new Pose3d(
                        new Translation3d(12.193, 2.527, 0),
                        Rotation3d.kZero);

        // Alliance zones
        private static final Rectangle2d Blue_Alliance_Zone = new Rectangle2d(
                        new Translation2d(FIELD_LEFT, FIELD_BOTTOM),
                        new Translation2d(ALLIANCE_ZONE_WIDTH, FIELD_TOP));
        private static final Rectangle2d Red_Alliance_Zone = new Rectangle2d(
                        new Translation2d(FIELD_RIGHT - ALLIANCE_ZONE_WIDTH, FIELD_BOTTOM),
                        new Translation2d(FIELD_RIGHT, FIELD_TOP));

        // Neutral zones
        private static final Rectangle2d Neutral_North_Zone = new Rectangle2d(
                        new Translation2d(4.626, FIELD_TOP),
                        Red_Hub_Position.getTranslation().toTranslation2d());
        private static final Rectangle2d Neutral_South_Zone = new Rectangle2d(
                        Blue_Hub_Position.getTranslation().toTranslation2d(),
                        new Translation2d(12.542, FIELD_BOTTOM));
        private static final Collection<Translation2d> Neutral_Zone_Centers = List.of(
                        Neutral_North_Zone.getCenter().getTranslation(),
                        Neutral_South_Zone.getCenter().getTranslation());

        // TODO: Set dead zones
        private static final Rectangle2d Red_Alliance_Dead_Zone = new Rectangle2d(new Translation2d(11.291, 8.225),
                        new Translation2d(12.548, -0.165));
        private static final Rectangle2d Blue_Alliance_Dead_Zone = new Rectangle2d(new Translation2d(5.204, 8.234),
                        new Translation2d(4, -0.164));

        public record TargetData(Pose3d targetPose, TargetType targetType) {
        }

        /**
         * Returns the passing position that the turret should aim towards based on the neutral-zone position of the robot.
         * If the robot is not in the neutral zone at all, returns null.
         * @param robotPosition
         * @return
         */
        public static TargetData GetTargetPosition(Pose2d robotPosition) {
                var robotTranslation = robotPosition.getTranslation();

                if (InEnemyScoringZone(robotPosition)) {
                        return new TargetData(new Pose3d(GetNeutralPassingPosition(robotPosition)),
                                        TargetType.kFarPassing);
                }

                if (InDeadZone(robotPosition)) {
                        return new TargetData(null, TargetType.kDead);
                }

                if (Robot.onRedAlliance()) {
                        if (Neutral_North_Zone.contains(robotTranslation)) {
                                return new TargetData(Red_North_Passing_Position, TargetType.kPassing);
                        } else if (Neutral_South_Zone.contains(robotTranslation)) {
                                return new TargetData(Red_South_Passing_Position, TargetType.kPassing);
                        } else {
                                return new TargetData(Red_Hub_Position, TargetType.kHub);
                        }
                } else {
                        if (Neutral_North_Zone.contains(robotTranslation)) {
                                return new TargetData(Blue_North_Passing_Position, TargetType.kPassing);
                        } else if (Neutral_South_Zone.contains(robotTranslation)) {
                                return new TargetData(Blue_South_Passing_Position, TargetType.kPassing);
                        } else {
                                return new TargetData(Blue_Hub_Position, TargetType.kHub);
                        }
                }
        }

        public static TargetData GetHubTargetFromTag(Double tagId, Pose3d tagPose) {
                if (Usable_Center_Hub_Targets.contains(tagId)) {
                        return new TargetData(tagPose.plus(Center_Hub_Target_Offset), TargetType.kHub);
                } else if (Usable_Left_Hub_Targets.contains(tagId)) {
                        return new TargetData(tagPose.plus(Left_Hub_Target_Offset), TargetType.kHub);
                } else if (Usable_Right_Hub_Targets.contains(tagId)) {
                        return new TargetData(tagPose.plus(Right_Hub_Target_Offset), TargetType.kHub);
                }

                return new TargetData(null, TargetType.kNone);
        }

        public static boolean InEnemyScoringZone(Pose2d robotPosition) {
                return Robot.onRedAlliance()
                                ? Blue_Alliance_Zone.contains(robotPosition.getTranslation())
                                : Red_Alliance_Zone.contains(robotPosition.getTranslation());
        }

        public static boolean InDeadZone(Pose2d robotPosition) {
                var translation = robotPosition.getTranslation();
                return Red_Alliance_Dead_Zone.contains(translation) || Blue_Alliance_Dead_Zone.contains(translation);
        }

        public static Pose2d GetNeutralPassingPosition(Pose2d robotPosition) {
                var translation = robotPosition.getTranslation().nearest(Neutral_Zone_Centers);

                return new Pose2d(translation, Rotation2d.kZero);
        }
}
