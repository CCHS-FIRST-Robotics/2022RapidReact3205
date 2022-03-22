package frc.robot.helper;

import frc.robot.Constants;

public class MPID {
    public double integral = 0;
    double previous;
    double previous_time;

    double prev_response = 0;

    public double k_p = 0.0;
    public double k_i = 0.0;
    public double k_d = 0.0;
    double decay = Math.exp(Math.log(0.5) / (Constants.INTERGRAL_HALFLIFE_T / Constants.MAIN_DT));

    public MPID(double s_p, double s_i, double s_d) {
        this.k_p = s_p;
        this.k_i = s_i;
        this.k_d = s_d;

        this.decay = Math.exp(Math.log(0.5) / (Constants.INTEGRAL_HALFLIFE_T_S / Constants.MAIN_DT));
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
        this.integral = this.integral * this.decay + delta * dt;

        double response = k_p * delta + k_i * this.integral + k_d * deriv;

        this.previous = delta;
        this.previous_time = current_time;

        response = Math.min(1, Math.max(-1, response));

        double rd = response - prev_response;
        if (rd > Constants.C_ACC_LIM * dt) {
            rd = Constants.C_ACC_LIM * dt;
        } else if (rd < -1 * Constants.C_ACC_LIM * dt) {
            rd = -1 * Constants.C_ACC_LIM * dt;
        }
        response = prev_response + rd;
        prev_response = response;

        return response;
    }
}