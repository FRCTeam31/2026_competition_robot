package org.prime.util;

import java.util.List;

/**
 * Helper class to normalize a data set
 */
public class Normalizer {
    private final double min;
    private final double max;

    public Normalizer(List<Double> data) {
        double tempMin = Double.MAX_VALUE;
        double tempMax = Double.MIN_VALUE;

        for (double v : data) {
            if (v < tempMin)
                tempMin = v;
            if (v > tempMax)
                tempMax = v;
        }

        this.min = tempMin;
        this.max = tempMax;
    }

    public double normalize(double value) {
        if (max == min)
            return 0;
        return (value - min) / (max - min);
    }

    public double denormalize(double value) {
        return value * (max - min) + min;
    }

    public List<Double> normalizeAll(List<Double> data) {
        return data.stream()
                .map(this::normalize)
                .toList();
    }
}
