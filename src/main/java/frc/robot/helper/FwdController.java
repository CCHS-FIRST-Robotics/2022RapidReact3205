package frc.robot.helper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.state.MainState;

public class FwdController {
    double v_max;
    double a_max;
    public double jerk;

    double target_v = 0;

    double previous_time;

    PID whl_c;

    public FwdController(double v_max, double a_max) {
        this.v_max = v_max;
        this.a_max = a_max;
        this.jerk = a_max / 1;
        this.target_v = 0;
        this.whl_c = new PID(Constants.C_BASE_GAIN, Constants.C_BASE_GAIN * 0.1, Constants.C_BASE_GAIN * 0.001);
        reset();
    }

    double getAcc(double clean_x, double clean_v) {
        // too short
        double min_dist = (2 * this.a_max * this.v_max / this.jerk) - (clean_v * clean_v) / (2 * this.a_max);
        if (clean_x > min_dist) {
            return this.a_max;
        }
        // Check if its too far
        boolean tf_triggered = false;
        if (Math.pow(clean_x / this.a_max, 0.5) < clean_x / this.v_max) {
            tf_triggered = true;
            return this.a_max;
        }
        SmartDashboard.putBoolean("FwdCont/tf triggered", tf_triggered);
        // Compute lin straight down
        double lsd = -1 * Math.pow(clean_v, 2) / (2 * clean_x);
        // Newtons method with convergence check
        double prev = a_max;
        double deriv = 0;
        double a = a_max;
        for (int i = 0; i < 10; i++) {
            if (a < 0) {
                return lsd;
            }
            double fx = (2 * a * a / this.jerk) - 2 * Math.pow(a * clean_x, 0.5) + clean_v;
            double fxp = (4 * a / this.jerk) + clean_v - Math.pow(clean_x / a, 0.5);
            a = a - fx / fxp;
            deriv = a - prev;
            prev = a;
            if (i > 7 && deriv > 0.2) {
                return lsd;
            }
        }
        return a;
    }

    public void reset() {
        this.previous_time = (double) System.currentTimeMillis() / 1000;
    }

    public double update(double current_v, double x) {
        double current_time = (double) System.currentTimeMillis() / 1000;
        double dt = current_time - this.previous_time;
        if (dt == 0) {
            dt = 0.0001;
        }
        double dir;
        if (x > 0) {
            dir = 1;
        } else {
            dir = -1;
        }
        double acc = getAcc(x * dir, current_v * dir) * dir;
        //SmartDashboard.putNumber("FwdCont/acc", acc);
        this.target_v = this.target_v + acc * dt;
        //SmartDashboard.putNumber("FwdCont/tv", this.target_v);
        this.target_v = Math.min(this.v_max, Math.max(-1 * this.v_max, target_v));
        //SmartDashboard.putNumber("FwdCont/tv", this.target_v);
        if (Math.abs(this.target_v) < 0.02 * this.v_max) {
            if (this.target_v > 0) {
                this.target_v = 0.02 * this.v_max;
            } else {
                this.target_v = -0.02 * this.v_max;
            }
        }
        return this.target_v;
    }
}
