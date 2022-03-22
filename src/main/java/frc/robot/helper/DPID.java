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
        this.prev_response = 0;
        this.previous = 0;
        this.hist_deriv = 0;
        this.previous_time = (double) System.currentTimeMillis() / 1000;
    }

    public double updateRaw(double t_radss, double c_radss) {
        // if its moving a way from the origin, limits to delta
        // otherwise boost gain and int but cap at 0
        double current_time = (double) System.currentTimeMillis() / 1000;
        double dt = current_time - this.previous_time;

        double[] temp_tunings = { k_p, k_i, k_d };

        boolean up = false;
        if (c_radss > 0) {
            if (t_radss > c_radss) {
                up = true;
            } else {
                up = false;
            }
        } else {
            if (t_radss < c_radss) {
                up = true;
            } else {
                up = false;
            }
        }
        if (up != true && Math.abs(t_radss - c_radss) > 0.2) {
            temp_tunings[0] = k_p * 1.1;
            temp_tunings[1] = k_i * 1.1;
            temp_tunings[2] = k_d * 1.1;
        }
        double delta = t_radss - c_radss;
        double deriv = 0;
        if (dt < Constants.MAIN_DT - 0.001) {
            dt = Constants.MAIN_DT;
        }
        else if (dt > Constants.MAX_DT) {
            dt = Constants.MAX_DT;
            deriv = (delta - this.previous) / dt;
        }
        else {
            deriv = (delta - this.previous) / dt;
        }
        this.hist_deriv = deriv * Constants.DERIV_FILTER + this.hist_deriv * (1 - Constants.DERIV_FILTER);

        this.integral = this.integral * this.decay + delta * dt;

        double response = k_p * delta + k_i * this.integral + k_d * hist_deriv;

        this.previous = delta;
        this.previous_time = current_time;

        if (Math.abs(response - this.prev_response) < Constants.DEADBAND) {
            response = this.prev_response;
        }

        // apply resp filter
        response = response * Constants.RESP_FILTER + this.prev_response * (1 - Constants.RESP_FILTER);

        if (up != true) {
            // if flip directions in response
            if (response > 0 && this.prev_response < 0) {
                response = 0;
            }
            if (response < 0 && this.prev_response > 0) {
                response = 0;
            }
        }
        this.prev_response = response;

        return response;
    }

    public double update(double delta) {
        // Recompute delta so it isn't changing too rapidly

        double current_time = (double) System.currentTimeMillis() / 1000;
        double dt = current_time - this.previous_time;

        double deriv = 0;
        if (dt < Constants.MAIN_DT - 0.001) {
            dt = Constants.MAIN_DT;
        }
        else if (dt > Constants.MAX_DT) {
            dt = Constants.MAX_DT;
            deriv = (delta - this.previous) / dt;
        }
        else {
            deriv = (delta - this.previous) / dt;
        }
        this.hist_deriv = deriv * Constants.DERIV_FILTER + this.hist_deriv * (1 - Constants.DERIV_FILTER);

        this.integral = this.integral * this.decay + delta * dt;

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