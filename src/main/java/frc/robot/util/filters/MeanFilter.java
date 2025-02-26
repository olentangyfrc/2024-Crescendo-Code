package frc.robot.util.filters;

import java.util.LinkedList;
import java.util.Queue;

public class MeanFilter {
    private final int windowSize;
    private final Queue<Double> vals;

    private double sum = 0;

    public MeanFilter(int windowSize) {
        this(windowSize, new double[] {});
    }

    public MeanFilter(int windowSize, double[] vals) {
        this.windowSize = windowSize;
        this.vals = new LinkedList<Double>();

        for(double val : vals) {
            calculate(val);
        }
    }

    public double calculate(double val) {
        vals.add(val);
        sum += val;

        while(vals.size() > windowSize) {
            sum -= vals.remove();
        }

        return sum / vals.size();
    }

    public int getNumVals() {
        return vals.size();
    }

    public int getWindowSize() {
        return windowSize;
    }
}
