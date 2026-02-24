package org.prime.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * <h1>IDWController</h1>
 * <p>A form of 2D interpolation used for mapping two inputs to a single output based on the distance from the
 * requested point to all data points in a 2D space, prioritizing closer points.</p>
 *
 * <h2>Explanation</h2>
 * <p>IDW, formerly known as Inverse Distance Weighting, is a form of multidimensional interpolation which, while can
 * be used in infinite many dimensions, is simplified to two dimensions here. IDW is commonly used for applications
 * such as geographical modeling and weather analysis, although it has some drawbacks namely slight skew towards large
 * collections of data points and regions of one value around secluded points.</p>
 *
 * <p>In order to interpolate values not in the data set, the controller loops through every entry in the data set and
 * calculates the distance between the requested point and the entry. Then, the controller calculates the weight for each
 * point following the formula {@code 1 / (the distance between the points)^(power)}. Next, the controller calculates
 * a weighted value following the formula {@code value in the entry * entry weight}. Finally, the controller then divides
 * the sum of all the weighted values by the sum of all the weights.</p>
 *
 * <h2>Implementation</h2>
 * <p>Because of the nature of IDW, it is important to have a somewhat even spread of data points and insure there are
 * no large gaps. Additionally, IDW cannot interpolate values further than its farthest point so it is important to get
 * data points at the extremes.</p>
 */
public class IDWController {
    /**
     * <h2>Entry</h2>
     * <p>An entry in a IDW data set. Requires two inputs, a and b, and an associated value.</p>
     *
     * <h3>Implementation</h3>
     * <p>Data does not need to be normalized, although manual normalization will not cause issues. Data is
     * automatically normalized on {@link IDWController} construction. The value component is not normalized, it will
     * output with the same scale as the values in your data set.</p>
     *
     * @param a The first controller input
     * @param b The second controller input
     * @param value The associated value at this coordinate {@code (a,b)}
     */
    public record Entry(double a, double b, double value) {}

    private final List<Entry> list;
    private final Normalizer normalizerA;
    private final Normalizer normalizerB;
    private final double power;

    /**
     * <h2>Constructor</h2>
     * <p>Create a IDW controller with a data set and custom power.</p>
     *
     * @param data The data set used in the construction of the IDW controller
     * @param power Power used in the IDW calculation
     */
    public IDWController(List<Entry> data, double power) {
        var listA = data.stream().map(Entry::a).toList();
        var listB = data.stream().map(Entry::b).toList();
        var listValue = data.stream().map(Entry::value).toList();

        this.normalizerA = new Normalizer(listA);
        this.normalizerB = new Normalizer(listB);
        this.power = power;

        var normListA = normalizerA.normalizeAll(listA);
        var normListB = normalizerB.normalizeAll(listB);

        this.list = new ArrayList<>(data.size());
        for (int i = 0; i < data.size(); i++) {
            list.add(new Entry(normListA.get(i), normListB.get(i), listValue.get(i)));
        }
    }

    /**
     * <h2>Constructor: Default</h2>
     * <p>Creates a IDW controller with the default power of 2.</p>
     */
    public IDWController(List<Entry> data) {
        this(data, 2);
    }

    /**
     * <h2>Calculate</h2>
     * <p>Calculate the output of the controller based on the two inputs a and b.</p>
     *
     * @param a The first controller input
     * @param b The second controller input
     * @return The calculated value of the controller
     */
    public double calculate(double a, double b) {
        var totalWeightedValue = 0.0;
        var totalWeight = 0.0;

        var normA = normalizerA.normalize(a);
        var normB = normalizerB.normalize(b);

        Optional<Entry> match = list.stream().filter(e -> e.a == normA && e.b == normB).findFirst();
        if (match.isPresent()) {
            return match.get().value;
        }

        for (Entry entry : list) {
            var A = entry.a;
            var B = entry.b;
            var VALUE = entry.value;

            var distA = A - normA;
            var distB = B - normB;
            var dist = Math.hypot(distA, distB);

            var weight = 1 / Math.pow(dist, power);
            var weightedValue = VALUE * weight;
            totalWeightedValue += weightedValue;
            totalWeight += weight;
        }

        return totalWeightedValue / totalWeight;
    }
}
