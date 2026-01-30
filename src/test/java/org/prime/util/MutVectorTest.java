package org.prime.util;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MutVectorTest {
    private MutVector mutVector;

    @BeforeEach
    public void setUp() {
        mutVector = new MutVector();
    }

    @Test
    public void testCreation_blank() {
        Assertions.assertEquals(0, mutVector.getX());
        Assertions.assertEquals(0, mutVector.getY());
        Assertions.assertEquals(0, mutVector.getZ());

        Assertions.assertEquals(0, mutVector.getMagnitude());
        Assertions.assertEquals(0, mutVector.getPitch());
        Assertions.assertEquals(0, mutVector.getYaw());
    }

    @Test
    public void testCreation_fromCartesian() {
        double x = 1;
        double y = 2;
        double z = 3;

        mutVector.fromCartesian(x, y, z);

        Assertions.assertEquals(x, mutVector.getX());
        Assertions.assertEquals(y, mutVector.getY());
        Assertions.assertEquals(z, mutVector.getZ());
    }

    @Test
    public void testCreation_fromPolar() {
        double magnitude = 1;
        double pitch = 2;
        double yaw = 3;

        mutVector.fromPolar(magnitude, pitch, yaw);

        Assertions.assertEquals(magnitude, mutVector.getMagnitude());
        Assertions.assertEquals(pitch, mutVector.getPitch());
        Assertions.assertEquals(yaw, mutVector.getYaw());
    }

    @Test
    public void testCreation_setCartesianBehavesAsFromCartesian() {
        double x = 1;
        double y = 2;
        double z = 3;

        MutVector testVector = new MutVector();
        testVector.setCartesian(x, y, z);

        mutVector.fromCartesian(x, y, z);

        Assertions.assertEquals(mutVector.getX(), testVector.getX());
        Assertions.assertEquals(mutVector.getY(), testVector.getY());
        Assertions.assertEquals(mutVector.getZ(), testVector.getZ());

        Assertions.assertEquals(mutVector.getMagnitude(), testVector.getMagnitude());
        Assertions.assertEquals(mutVector.getPitch(), testVector.getPitch());
        Assertions.assertEquals(mutVector.getYaw(), testVector.getYaw());
    }

    @Test
    public void testCreation_setPolarBehavesAsFromPolar() {
        double magnitude = 1;
        double pitch = 2;
        double yaw = 3;

        MutVector testVector = new MutVector();
        testVector.setPolar(magnitude, pitch, yaw);

        mutVector.setPolar(magnitude, pitch, yaw);

        Assertions.assertEquals(mutVector.getX(), testVector.getX());
        Assertions.assertEquals(mutVector.getY(), testVector.getY());
        Assertions.assertEquals(mutVector.getZ(), testVector.getZ());

        Assertions.assertEquals(mutVector.getMagnitude(), testVector.getMagnitude());
        Assertions.assertEquals(mutVector.getPitch(), testVector.getPitch());
        Assertions.assertEquals(mutVector.getYaw(), testVector.getYaw());
    }

    @Test
    public void testSet_x() {
        double x_before = 1;
        double x_after = 2;

        mutVector.fromCartesian(x_before, 0, 0);
        mutVector.setX(x_after);

        Assertions.assertEquals(x_after, mutVector.getX());
    }

    @Test
    public void testSet_y() {
        double y_before = 1;
        double y_after = 2;

        mutVector.fromCartesian(0, y_before, 0);
        mutVector.setY(y_after);

        Assertions.assertEquals(y_after, mutVector.getY());
    }

    @Test
    public void testSet_z() {
        double z_before = 1;
        double z_after = 2;

        mutVector.fromCartesian(0, 0, z_before);
        mutVector.setZ(z_after);

        Assertions.assertEquals(z_after, mutVector.getZ());
    }

    @Test
    public void testSet_magnitude() {
        double magnitude_before = 1;
        double magnitude_after = 2;

        mutVector.fromPolar(magnitude_before, 0, 0);
        mutVector.setMagnitude(magnitude_after);

        Assertions.assertEquals(magnitude_after, mutVector.getMagnitude());
    }

    @Test
    public void testSet_pitch() {
        double pitch_before = 1;
        double pitch_after = 2;

        mutVector.fromPolar(1, pitch_before, 0);
        mutVector.setPitch(pitch_after);

        Assertions.assertEquals(pitch_after, mutVector.getPitch());
    }

    @Test
    public void testSet_yaw() {
        double yaw_before = 1;
        double yaw_after = 2;

        mutVector.fromPolar(1, 0, yaw_before);
        mutVector.setYaw(yaw_after);

        Assertions.assertEquals(yaw_after, mutVector.getYaw());
    }

    @Test
    public void testArithmetics_addition() {
        double x_1 = 1;
        double y_1 = 2;
        double z_1 = 3;
        double x_2 = 4;
        double y_2 = 5;
        double z_2 = 6;

        MutVector testVector = new MutVector().fromCartesian(x_2, y_2, z_2);

        mutVector.fromCartesian(x_1, y_1, z_1);
        mutVector.plus(testVector);

        Assertions.assertEquals(x_1 + x_2, mutVector.getX());
        Assertions.assertEquals(y_1 + y_2, mutVector.getY());
        Assertions.assertEquals(z_1 + z_2, mutVector.getZ());
    }

    @Test
    public void testArithmetics_subtraction() {
        double x_1 = 1;
        double y_1 = 2;
        double z_1 = 3;
        double x_2 = 4;
        double y_2 = 5;
        double z_2 = 6;

        MutVector testVector = new MutVector().fromCartesian(x_2, y_2, z_2);

        mutVector.fromCartesian(x_1, y_1, z_1);
        mutVector.minus(testVector);

        Assertions.assertEquals(x_1 - x_2, mutVector.getX());
        Assertions.assertEquals(y_1 - y_2, mutVector.getY());
        Assertions.assertEquals(z_1 - z_2, mutVector.getZ());
    }
}
