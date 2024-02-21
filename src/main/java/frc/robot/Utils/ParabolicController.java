package frc.robot.Utils;

public class ParabolicController {
    double p;

    public ParabolicController() {
        this.p = 0;
    }

    public ParabolicController(double p) {
        this.p = 0.01 * p;
    }

    public void setP(double p) {
        this.p = 0.01 * p;
    }

    public double calc(double error) {
        double ans = p * Math.pow(error, 2);

        if (error < 0) {
            return -ans;
        } else
            return ans;
    }
}
