package frc.robot;

import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Static class containing field target positions and helper methods. Coordinates derived from Pathplanner field model.
 * Field origin is bottom-left corner with Blue Alliance on the left side when viewed from above.
 */
public class FieldTargets {
    public static final double FIELD_TOP = 8.084; // meters
    public static final double FIELD_MIDDLE = FIELD_TOP / 2; // meters
    public static final double FIELD_BOTTOM = 0; // meters
    public static final double FIELD_LEFT = 0; // meters
    public static final double FIELD_RIGHT = 16.561; // meters
    public static final double HUB_HEIGHT = 1.8288; // meters (72")

    private static final Pose3d Blue_Hub_Position = new Pose3d(
            new Translation3d(4.618, 4.035, 1.8288), // 1.8288m (72") is the height of the top rim
            Rotation3d.kZero);
    private static final Pose3d Red_Hub_Position = new Pose3d(
            new Translation3d(11.919, 4.035, 1.8288),
            Rotation3d.kZero);

    public static Pose3d GetHubPosition() {
        return Robot.onBlueAlliance()
                ? Blue_Hub_Position
                : Red_Hub_Position;
    }

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

    private static final Rectangle2d Blue_Alliance_Zone = new Rectangle2d(
            new Translation2d(FIELD_LEFT, FIELD_BOTTOM),
            new Translation2d(3.998, FIELD_TOP));
    private static final Rectangle2d Red_Alliance_Zone = new Rectangle2d(
            new Translation2d(12.542, FIELD_BOTTOM),
            new Translation2d(FIELD_RIGHT, FIELD_TOP));
    private static final Rectangle2d Neutral_North_Zone = new Rectangle2d(
            new Translation2d(4.626, FIELD_TOP),
            Red_Hub_Position.getTranslation().toTranslation2d());
    private static final Rectangle2d Neutral_South_Zone = new Rectangle2d(
            Blue_Hub_Position.getTranslation().toTranslation2d(),
            new Translation2d(12.542, FIELD_BOTTOM));
    private static final Collection<Translation2d> Neutral_Zone_Centers = List.of(
            Neutral_North_Zone.getCenter().getTranslation(),
            Neutral_South_Zone.getCenter().getTranslation());

    /**
     * Returns the passing position that the turret should aim towards based on the neutral-zone position of the robot.
     * If the robot is not in the neutral zone at all, returns null.
     * @param robotAlliance
     * @param robotPosition
     * @return
     */
    public static Pose3d GetPassingPosition(Pose2d robotPosition) {
        var robotTranslation = robotPosition.getTranslation();

        if (InEnemyScoringZone(robotPosition)) {
            return new Pose3d(GetNeutralPassingPosition(robotPosition));
        }

        if (Robot.onRedAlliance()) {
            if (Neutral_North_Zone.contains(robotTranslation)) {
                return Red_North_Passing_Position;
            } else if (Neutral_South_Zone.contains(robotTranslation)) {
                return Red_South_Passing_Position;
            } else {
                return null;
            }
        } else {
            if (Neutral_North_Zone.contains(robotTranslation)) {
                return Blue_North_Passing_Position;
            } else if (Neutral_South_Zone.contains(robotTranslation)) {
                return Blue_South_Passing_Position;
            } else {
                return null;
            }
        }
    }

    public static boolean InEnemyScoringZone(Pose2d robotPosition) {
        return Robot.onRedAlliance()
                ? Blue_Alliance_Zone.contains(robotPosition.getTranslation())
                : Red_Alliance_Zone.contains(robotPosition.getTranslation());
    }

    public static Pose2d GetNeutralPassingPosition(Pose2d robotPosition) {
        var translation = robotPosition.getTranslation().nearest(Neutral_Zone_Centers);

        return new Pose2d(translation, Rotation2d.kZero);
    }
}
