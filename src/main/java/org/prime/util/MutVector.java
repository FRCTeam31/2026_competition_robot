package org.prime.util;

/**
 * Vector object that perminatly changes with addition and subtraction,
 * and can be fully set to a new vector with a single call.
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

        _x = magnitude * Math.cos(pitchRad) * Math.cos(yawRad);
        _y = magnitude * Math.sin(pitchRad);
        _z = magnitude * Math.cos(pitchRad) * Math.sin(yawRad);

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
     * @return The pitch 
     */
    public double getPitch() {
        return Math.atan(_z / Math.sqrt(Math.pow(_x, 2) + Math.pow(_y, 2)));
    }

    /**
     * Gets the yaw of the Vector
     * @return The yaw 
     */
    public double getYaw() {
        return Math.atan(_y / _x);
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

}
