package frc.robot.helper;

public class Curve {
    double time;
    double init_prop;
    double exp;
    double action_time;

    public Curve(double time, double init_prop, double exp) {
        this.time = time;
        this.init_prop = init_prop;
        this.exp = exp;
        this.action_time = (double) System.currentTimeMillis() / 1000;
    }

    public double getProp(double current_prop) {
        if (Math.abs(current_prop) < 0.1) {
            this.action_time = (double) System.currentTimeMillis() / 1000;
        }
        double dt = (double) System.currentTimeMillis() / 1000;
        dt = dt - this.action_time;
        // dt = this.time = 1
        double y = (1 - this.init_prop) * Math.pow(dt / this.time, this.exp) + this.init_prop;
        y = Math.max(0, Math.min(1, y));
        return y * current_prop;
    }
}
