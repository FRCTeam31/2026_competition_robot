package frc.robot;

import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldTargets {
    private static final Pose3d Red_Hub_Position = new Pose3d(); // TODO: add proper coordinates
    private static final Pose3d Blue_Hub_Position = new Pose3d(); // TODO: add proper coordinates

    public static Pose3d GetHubPosition() {
        return Robot.onBlueAlliance()
                ? Blue_Hub_Position
                : Red_Hub_Position;
    }

    private static final Pose3d Red_North_Passing_Position = new Pose3d(); // TODO: add proper coordinates
    private static final Pose3d Red_South_Passing_Position = new Pose3d(); // TODO: add proper coordinates
    private static final Pose3d Blue_North_Passing_Position = new Pose3d(); // TODO: add proper coordinates
    private static final Pose3d Blue_South_Passing_Position = new Pose3d(); // TODO: add proper coordinates

    private static final Rectangle2d Blue_Scoring_Zone = new Rectangle2d(null, null); // TODO: add proper coordinates
    private static final Rectangle2d Red_Scoring_Zone = new Rectangle2d(null, null); // TODO: add proper coordinates
    private static final Rectangle2d Neutral_North_Zone = new Rectangle2d(null, null); // TODO: add proper coordinates
    private static final Rectangle2d Neutral_South_Zone = new Rectangle2d(null, null); // TODO: add proper coordinates
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
                ? Blue_Scoring_Zone.contains(robotPosition.getTranslation())
                : Red_Scoring_Zone.contains(robotPosition.getTranslation());
    }

    public static Pose2d GetNeutralPassingPosition(Pose2d robotPosition) {
        var translation = robotPosition.getTranslation().nearest(Neutral_Zone_Centers);

        return new Pose2d(translation, Rotation2d.kZero);
    }
}
