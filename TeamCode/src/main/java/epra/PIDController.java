package epra;

public class PIDController {

    private double k_p, k_i, k_d;

    private double p,i,d;

    private long saveTime;
    private double saveError;

    public PIDController(double k_p, double k_i, double k_d) {
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
        reset();
    }

    public void tuneP(double k_p) { this.k_p = k_p; }
    public void tuneI(double k_i) { this.k_i = k_i; }
    public void tuneD(double k_d) { this.k_d = k_d; }

    public double runPID(double current, double target) {
        double currentError = target - current;
        if (saveError == 0) { saveError = currentError; }
        long currentTime = System.currentTimeMillis();

        p = k_p * currentError;
        i += k_i * (currentError * (currentTime - saveTime));
        if (Math.abs(i) > 1) { i = Math.signum(i); }
        d = k_d * (currentError - saveError) / (currentTime - saveTime);

        saveError = currentError;
        saveTime = currentTime;

        return p + i + d;
    }

    public void reset() {
        p = 0;
        i = 0;
        d = 0;
        saveTime = System.currentTimeMillis();
        saveError = 0;
    }
}
