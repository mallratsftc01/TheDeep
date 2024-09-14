package epra.math.statistics;
/**A class that adds many statistical functions for many uses.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Statistics {

    public Statistics() {}

    /**@param set A set of values.
     * @return The average value of the set.*/
    public static double average(double[] set) {
        double v = 0.0;
        for (double d : set) { v += d; }
        return v / (double) set.length;
    }
    /**Finds the derivative array of an array.
     * @param set The array to find the derivative of.
     * @return The array of the derivative.*/
    public static double[] differentiate(double[] set) {
        double[] r = new double[set.length];
        r[0] = set[1] - set[0];
        r[set.length - 1] = r[set.length - 1] - r[set.length - 2];
        for (int i = 1; i < set.length - 1; i++) {
            r[i] = (r[i + 1] - r[i - 1]) / 2;
        }
        return r;
    }
}
