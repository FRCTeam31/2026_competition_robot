package org.prime.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.turret.TurretMap;

/**
 * Vector object that permanently changes with addition and subtraction.
 * Can be fully set to a new vector with a single call.
 */
public class MutVector {
    private double _x;
    private double _y;
    private double _z;

    /**
     * Creates a blank vector
     */
    public MutVector() {
        _x = 0;
        _y = 0;
        _z = 0;
    }

    /**
     * Creates the vector from Cartesian corrdinates
     * @param x The x corrdinate of the vector
     * @param y The y corrdinate of the vector
     * @param z The z corrdinate of the vector
     * @return The constructed vector
     */
    public MutVector fromCartesian(double x, double y, double z) {
        _x = x;
        _y = y;
        _z = z;

        return this;
    }

    /**
     * Creates the vector from Polar corrdinates
     * @param magnitude The magnitude of the vector
     * @param pitch The pitch of the vector
     * @param yaw The yaw of the vector
     * @return The constructed vector
     */
    public MutVector fromPolar(double magnitude, double pitch, double yaw) {
        double pitchRad = Math.toRadians(pitch);
        double yawRad = Math.toRadians(yaw);

        _x = magnitude * Math.sin(pitchRad) * Math.cos(yawRad);
        _y = magnitude * Math.sin(pitchRad) * Math.sin(yawRad);
        _z = magnitude * Math.cos(pitchRad);

        return this;
    }

    /**
     * Gets the x value of the vector
     * @return The x value
     */
    public double getX() {
        return _x;
    }

    /**
     * Gets the y value of the vector
     * @return The y value
     */
    public double getY() {
        return _y;
    }

    /**
     * Gets the z value of the vector
     * @return The z value
     */
    public double getZ() {
        return _z;
    }

    /**
     * Gets the magnitude of the Vector
     * @return The magnitude 
     */
    public double getMagnitude() {
        return Math.sqrt(Math.pow(_x, 2) + Math.pow(_y, 2) + Math.pow(_z, 2));
    }

    /**
     * Gets the pitch of the Vector
     * @return The pitch (0 for zero-magnitude vectors)
     */
    public double getPitch() {
        double magnitude = getMagnitude();
        if (magnitude == 0)
            return 0; // Undefined for zero vector, return 0
        return Math.toDegrees(Math.acos(_z / magnitude));
    }

    /**
     * Gets the yaw of the Vector
     * @return The yaw 
     */
    public double getYaw() {
        return Math.toDegrees(Math.atan2(_y, _x));
    }

    /**
     * Sets the vector using polar values
     * @param magnitude The magnitude value
     * @param pitch The pitch value
     * @param yaw The yaw value
     */
    public void setPolar(double magnitude, double pitch, double yaw) {
        this.fromPolar(magnitude, pitch, yaw);
    }

    /**
     * Sets the vector using Cartesian values
     * @param x The x value
     * @param y The y value
     * @param z The z value
     */
    public void setCartesian(double x, double y, double z) {
        this.fromCartesian(x, y, z);
    }

    /**
     * Sets the magnitude
     * @param magnitude The magnitude value
     */
    public void setMagnitude(double magnitude) {
        this.fromPolar(magnitude, this.getPitch(), this.getYaw());
    }

    /**
     * Sets the pitch
     * @param pitch The pitch value
     */
    public void setPitch(double pitch) {
        this.fromPolar(this.getMagnitude(), pitch, this.getYaw());
    }

    /**
     * Sets the yaw
     * @param yaw The yaw value
     */
    public void setYaw(double yaw) {
        this.fromPolar(this.getMagnitude(), this.getPitch(), yaw);
    }

    public void setX(double x) {
        this.fromCartesian(x, _y, _z);
    }

    public void setY(double y) {
        this.fromCartesian(_x, y, _z);
    }

    public void setZ(double z) {
        this.fromCartesian(_x, _y, z);
    }

    /**
     * Adds a vector to the curent vector
     * @param vector The vector that's being added
     * @return The vector outcome
     */
    public MutVector plus(MutVector vector) {
        double x = _x + vector.getX();
        double y = _y + vector.getY();
        double z = _z + vector.getZ();

        return this.fromCartesian(x, y, z);
    }

    /**
     * Subtracts a vector from the curent vector
     * @param vector The vector that's being subtracted
     * @return The vector outcome
     */
    public MutVector minus(MutVector vector) {
        double x = _x - vector.getX();
        double y = _y - vector.getY();
        double z = _z - vector.getZ();

        return this.fromCartesian(x, y, z);
    }

    /**
     * Sets the vector to shoot from a source pose to a target pose
     * @param sourcePose The source pose
     * @param targetPose The target pose
     * @param minAngle The minimum angle of the projectile
     * @param maxAngle The maximum angle of the projectile
     * @param minSpeed The minimum speed of the projectile
     * @param maxSpeed The maximum speed of the projectile
     * @throws Exception If no valid shot solution is found
     */
    public void setToTargetVector(Pose3d sourcePose, Pose3d targetPose,
            double minAngle, double maxAngle,
            double minSpeed, double maxSpeed) throws Exception {

        // this code is largely generated by claude
        // somewhat reviewed by me but
        // TODO: double check and improve

        // calculate angle and distance (x, y)
        double deltaX = targetPose.getX() - sourcePose.getX();
        double deltaY = targetPose.getY() - sourcePose.getY();
        double yaw = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double distance = Math.hypot(deltaX, deltaY);

        // calculate height difference (z)
        double shooterHeight = TurretMap.TURRET_HEIGHT_ABOVE_GROUND;
        double targetHeight = targetPose.getZ() + TurretMap.HUB_OVERSHOOT_HEIGHT;
        double deltaH = targetHeight - shooterHeight;

        // utilize optimal angle formula (45° + (arctan(deltaH / distance) / 2)
        double optimalFromHorizontal = 45.0 + (Math.toDegrees(Math.atan2(deltaH, distance)) / 2.0);

        // subtract 90 to convert from "from horizontal" to "from vertical" for turret math
        double angle = Math.max(minAngle, Math.min(maxAngle, 90 - optimalFromHorizontal));

        // convert back to "from horizontal" for physics math by subtracting from 90
        double physicsAngleRad = Math.toRadians(90.0 - angle);
        double cos = Math.cos(physicsAngleRad);
        double tan = Math.tan(physicsAngleRad);

        // calculate initial exit speed
        double denominator = 2 * cos * cos * (distance * tan - deltaH);

        if (denominator <= 0) {
            throw new Exception("geometry impossible at optimal angle");
        }

        double vSquared = PhysicsConstants.GRAVITY * distance * distance / denominator;

        if (vSquared <= 0) {
            throw new Exception("negative vSquared");
        }

        double v = Math.sqrt(vSquared);

        // validate against limits
        if (v < minSpeed || v > maxSpeed) {
            throw new Exception(String.format(
                    "required speed %.2f m/s outside [%.2f, %.2f]",
                    v, minSpeed, maxSpeed));
        }

        setPolar(v, angle, yaw);
    }

    public Time getTimeToTarget(double distanceToTarget) {
        double horizontalVelocity = Math.sqrt(Math.pow(_x, 2) + Math.pow(_y, 2));

        if (horizontalVelocity == 0) {
            return Units.Seconds.of(-1);
        }

        return Units.Seconds.of(distanceToTarget / horizontalVelocity);
    }

    // TODO: Add unit tests
    public Translation3d getTranslation3d() {
        return new Translation3d(_x, _y, _z);
    }
}
