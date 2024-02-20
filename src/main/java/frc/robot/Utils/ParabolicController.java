package frc.robot.Utils;

public class ParabolicController {
    double p;

    public ParabolicController() {
        this.p = 0;
    }

    public ParabolicController(double p) {
        this.p = p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double calc(double x) {
        double ans = p * Math.pow(x, 2);

        if (x < 0) {
            return -ans;
        } else
            return ans;
    }
}
