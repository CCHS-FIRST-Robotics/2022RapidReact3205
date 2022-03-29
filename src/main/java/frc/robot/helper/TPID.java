package frc.robot.helper;

import frc.robot.Constants;

public class TPID {
    public double integral = 0;
    double previous;
    double previous_time;

    double prev_response = 0;
    double hist_deriv = 0;

    public double k_p = 0.1;
    public double k_i = 0.01;
    public double k_d = 0.01;
    double decay = Math.exp(Math.log(0.5) / (Constants.INTEGRAL_HALFLIFE_T_S / Constants.MAIN_DT));

    public TPID(double s_p, double s_i, double s_d) {
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
        this.prev_response = 0;
        this.previous = 0;
        this.hist_deriv = 0;
        this.previous_time = (double) System.currentTimeMillis() / 1000;
    }

    public void softReset() {
        this.integral = 0;
    }

    public void integralClamp() {
        if (this.integral * this.k_i > Constants.INT_LIMIT) {
            this.integral = Constants.INT_LIMIT / this.k_i;
        }
        if (this.integral * this.k_i < -1 * Constants.INT_LIMIT) {
            this.integral = -1 * Constants.INT_LIMIT / this.k_i;
        }
    }

    public double update(double delta) {
        // Recompute delta so it isn't changing too rapidly

        double current_time = (double) System.currentTimeMillis() / 1000;
        double dt = current_time - this.previous_time;

        double deriv = 0;
        if (dt < Constants.MAIN_DT - 0.001) {
            dt = Constants.MAIN_DT;
        } else if (dt > Constants.MAX_DT) {
            dt = Constants.MAX_DT;
            deriv = (delta - this.previous) / dt;
        } else {
            deriv = (delta - this.previous) / dt;
        }
        this.hist_deriv = deriv * Constants.DERIV_FILTER + this.hist_deriv * (1 - Constants.DERIV_FILTER);

        if (this.hist_deriv > Constants.DERIV_LIMIT) {
            this.hist_deriv = Constants.DERIV_LIMIT;
        }
        if (this.hist_deriv < -1 * Constants.DERIV_LIMIT) {
            this.hist_deriv = -1 * Constants.DERIV_LIMIT;
        }

        this.integral = this.integral * this.decay + delta * dt;
        integralClamp();

        double response = k_p * delta + k_i * this.integral + k_d * hist_deriv;

        this.previous = delta;
        this.previous_time = current_time;

        if (Math.abs(response - this.prev_response) < Constants.DEADBAND) {
            response = this.prev_response;
        }

        // apply resp filter
        response = response * Constants.RESP_FILTER + this.prev_response * (1 - Constants.RESP_FILTER);

        this.prev_response = response;

        return response;
    }
}