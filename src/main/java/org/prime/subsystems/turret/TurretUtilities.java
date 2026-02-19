package org.prime.subsystems.turret;

import org.prime.util.MutVector;
import org.prime.util.PhysicsConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Whitelabel turret utility functions for ballistics and sensor pose calculations.
 * These are pure math operations with no subsystem or hardware dependencies.
 */
public class TurretUtilities {

    /**
     * Calculates the aim vector (velocity, pitch, yaw) needed to launch a projectile
     * from a source pose to a target pose using optimal-angle projectile physics.
     *
     * Uses the optimal angle formula: 45° + (arctan(deltaH / distance) / 2)
     * and solves for the required launch velocity.
     *
     * @param result              The MutVector to write the result into (magnitude = speed, pitch/yaw
     *                            in degrees)
     * @param sourcePose          The 3D pose of the launcher
     * @param targetPose          The 3D pose of the target
     * @param shooterHeight       The height of the shooter above the ground (meters)
     * @param targetOvershootHeight Additional height to add to the target (e.g., overshoot margin)
     * @param minAngle            The minimum allowed launch angle (degrees, from vertical — 0° is
     *                            straight up)
     * @param maxAngle            The maximum allowed launch angle (degrees, from vertical)
     * @param minSpeed            The minimum allowed launch speed (m/s)
     * @param maxSpeed            The maximum allowed launch speed (m/s)
     * @throws Exception If no valid shot solution exists (impossible geometry, speed out of bounds)
     */
    public static void calculateAimVector(MutVector result, Pose3d sourcePose, Pose3d targetPose,
            double shooterHeight, double targetOvershootHeight,
            double minAngle, double maxAngle, double minSpeed, double maxSpeed) throws Exception {

        // Calculate horizontal angle and distance (x, y)
        double deltaX = targetPose.getX() - sourcePose.getX();
        double deltaY = targetPose.getY() - sourcePose.getY();
        double yaw = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double distance = Math.hypot(deltaX, deltaY);

        // Calculate height difference (z)
        double targetHeight = targetPose.getZ() + targetOvershootHeight;
        double deltaH = targetHeight - shooterHeight;

        // Utilize optimal angle formula: 45° + (arctan(deltaH / distance) / 2)
        double optimalFromHorizontal = 45.0 + (Math.toDegrees(Math.atan2(deltaH, distance)) / 2.0);

        // Convert from "from horizontal" to "from vertical" for turret math, and clamp
        double angle = Math.max(minAngle, Math.min(maxAngle, 90 - optimalFromHorizontal));

        // Convert back to "from horizontal" for physics math
        double physicsAngleRad = Math.toRadians(90.0 - angle);
        double cos = Math.cos(physicsAngleRad);
        double tan = Math.tan(physicsAngleRad);

        // Calculate initial exit speed
        double denominator = 2 * cos * cos * (distance * tan - deltaH);

        if (denominator <= 0) {
            throw new Exception("geometry impossible at optimal angle");
        }

        double vSquared = PhysicsConstants.GRAVITY * distance * distance / denominator;

        if (vSquared <= 0) {
            throw new Exception("negative vSquared");
        }

        double v = Math.sqrt(vSquared);

        // Validate against speed limits
        if (v < minSpeed || v > maxSpeed) {
            throw new Exception(String.format(
                    "required speed %.2f m/s outside [%.2f, %.2f]",
                    v, minSpeed, maxSpeed));
        }

        result.setPolar(v, angle, yaw);
    }

    /**
     * Calculates the 3D pose of a sensor (e.g., Limelight) mounted on a turret,
     * relative to the robot's center on the ground.
     *
     * Accounts for:
     * - The turret's rotation origin offset from the robot center
     * - The current turret rotation angle around the Z-axis
     * - The sensor's fixed offset from the turret rotation center
     * - The sensor's fixed orientation (pitch, yaw, roll) relative to the turret
     *
     * @param turretOriginFromRobotCenter The turret's rotation center position relative to robot
     *                                    center
     * @param sensorOffsetX               Sensor X offset from turret center (forward, meters)
     * @param sensorOffsetY               Sensor Y offset from turret center (left, meters)
     * @param sensorOffsetZ               Sensor Z offset from turret center (up, meters)
     * @param sensorPitch                 Sensor fixed pitch angle (radians)
     * @param sensorYaw                   Sensor fixed yaw angle (radians)
     * @param sensorRoll                  Sensor fixed roll angle (radians)
     * @param turretRotationRadians       Current turret rotation around Z-axis (radians)
     * @return Pose3d of the sensor relative to the robot center on the ground
     */
    public static Pose3d calculateSensorPose(
            Translation3d turretOriginFromRobotCenter,
            Transform3d sensorTransformFromTurretCenter,
            double turretRotationRadians) {

        // Rotate sensor XY offset around turret Z-axis
        double rotatedX = sensorTransformFromTurretCenter.getX() * Math.cos(turretRotationRadians)
                - sensorTransformFromTurretCenter.getY() * Math.sin(turretRotationRadians);
        double rotatedY = sensorTransformFromTurretCenter.getX() * Math.sin(turretRotationRadians)
                + sensorTransformFromTurretCenter.getY() * Math.cos(turretRotationRadians);

        Translation3d sensorOffsetFromTurretCenter = new Translation3d(
                rotatedX,
                rotatedY,
                sensorTransformFromTurretCenter.getZ());

        // Combine turret origin + rotated sensor offset
        Translation3d sensorPositionFromRobotCenter = turretOriginFromRobotCenter
                .plus(sensorOffsetFromTurretCenter);

        // Sensor rotation includes fixed orientation + turret rotation on yaw axis
        Rotation3d sensorRotation = new Rotation3d(
                sensorTransformFromTurretCenter.getRotation().getX(),
                sensorTransformFromTurretCenter.getRotation().getY(),
                sensorTransformFromTurretCenter.getRotation().getZ() + turretRotationRadians);

        return new Pose3d(sensorPositionFromRobotCenter, sensorRotation);
    }
}
