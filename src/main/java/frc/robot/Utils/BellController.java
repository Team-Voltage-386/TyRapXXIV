package frc.robot.Utils;

public class BellController {
    double max;
    double p;

    public BellController() {
        this.max = 0;
        this.p = 0;
    }

    public BellController(double max, double p) {
        this.max = max;
        this.p = p;
    }

    public void setMax(double max) {
        this.max = max;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double calc(double error) {
        double ans = max - max * Math.pow(1 + 0.01 * p, -Math.pow(error, 2));
        if (error < 0) {
            return -ans;
        } else
            return ans;
    }
}
