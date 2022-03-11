package frc.robot.helper;

import frc.robot.Constants;

public class DPID {
    public double integral = 0;
    double previous;
    double previous_time;

    double prev_response = 0;
    double hist_deriv = 0;

    public double k_p = 0.1;
    public double k_i = 0.01;
    public double k_d = 0.01;
    double decay = Math.exp(Math.log(0.5) / (Constants.INTEGRAL_HALFLIFE_T_S / Constants.MAIN_DT));

    public DPID(double s_p, double s_i, double s_d) {
        this.k_p = s_p;
        this.k_i = s_i;
        this.k_d = s_d;

        this.decay = Math.exp(Math.log(0.5) / (Constants.INTERGRAL_HALFLIFE_T / Constants.MAIN_DT));
        reset();
    }

    public void setGain(double new_p) {
        this.k_p = new_p;
    }

    public void reset() {
        this.integral = 0;
        this.previous_time = (double) System.currentTimeMillis() / 1000;
    }

    public double update(double delta) {
        // Recompute delta so it isn't changing too rapidly

        double current_time = (double) System.currentTimeMillis() / 1000;
        double dt = current_time - this.previous_time;

        if (dt == 0) {
            dt = 0.0001;
        }
        if (dt > Constants.MAX_DT) {
            dt = Constants.MAX_DT;
        }

        double d2 = (delta - this.previous) / dt;
        if (d2 > Constants.MAX_M_SP_ACC) {
            delta = this.previous + Constants.MAX_M_SP_ACC;
        } else {
            delta = this.previous + Constants.MAX_M_SP_ACC;
        }

        double deriv = (delta - this.previous) / dt;
        this.hist_deriv = deriv * Constants.DERIV_FILTER + this.hist_deriv * (1 - Constants.DERIV_FILTER);

        this.integral = this.integral * this.decay + delta * dt;

        double response = k_p * delta + k_i * this.integral + k_d * hist_deriv;

        this.previous = delta;
        this.previous_time = current_time;

        if (Math.abs(response - this.prev_response) < Constants.DEADBAND) {
            response = this.prev_response;
        }

        this.prev_response = response;

        return response;
    }
}