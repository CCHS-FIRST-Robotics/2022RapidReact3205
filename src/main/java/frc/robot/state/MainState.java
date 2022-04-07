package frc.robot.state;

import java.lang.*;

public class MainState {
    Kinematics Phy = new Kinematics();
    Wheel Whl = new Wheel();
    Storage Sto = new Storage();
    Shooter Sho = new Shooter();
    Beam b = new Beam();
    Lime l = new Lime();

    public void predict(double dt) {
        this.Whl.predict(dt, this.Phy.Val.heading, this.Phy.Val.acc, this.Phy.Val.ang_vel);
        this.Phy.predict(dt);
    }

    /**
     * Kalman Update. Main method of fusing sensor values by comparing variances.
     * 
     * @param current_val Current value from state.
     * @param current_var Current variance from state.
     * @param sensed_val  Sensed value from sensor.
     * @param sensed_var  Sensed variance from sensor.
     * @return Double len 2 array. Element 0 is fused value, Element 1 is fused
     *         variance.
     */
    public double[] kalmanUpdate(double current_val, double current_var, double sensed_val, double sensed_var) {
        double kalman_gain = current_var / (current_var + sensed_var + 0.0001);
        double new_val = kalman_gain * sensed_val + (1 - kalman_gain) * current_val;
        double new_var = kalman_gain * sensed_var + (1 - kalman_gain) * current_var;
        if (Double.isNaN(new_val) || Double.isNaN(new_var)) {
            double[] new_1 = { current_val, current_var };
            return new_1;
        } else {
            double[] new_1 = { new_val, new_var };
            return new_1;
        }
    }

    public double[] kalman2Update(double[] current_val, double current_var, double[] sensed_val, double sensed_var) {
        double[] x = kalmanUpdate(current_val[0], current_var, sensed_val[0], sensed_var);
        double[] y = kalmanUpdate(current_val[1], current_var, sensed_val[1], sensed_var);
        double[] z = { x[0], y[0], x[1] };
        return z;
    }

    /**
     * Kalman Update. Main method of fusing sensor values by comparing variances.
     * Special version for wrapping angular values like heading.
     * 
     * @param current_val Current value from state.
     * @param current_var Current variance from state.
     * @param sensed_val  Sensed value from sensor.
     * @param sensed_var  Sensed variance from sensor.
     * @return Double len 2 array. Element 0 is fused value, Element 1 is fused
     *         variance.
     */
    public double[] kalmanAngleUpdate(double current_val, double current_var, double sensed_val, double sensed_var) {
        double[] candidate_sval = { sensed_val - 2 * Math.PI, sensed_val, sensed_val + 2 * Math.PI };
        double new_sval = sensed_val;
        double closest = 8 * Math.PI;
        for (int i = 0; i < 3; i++) {
            double dist = Math.abs(candidate_sval[i] - current_val);
            if (dist < closest) {
                closest = dist;
                new_sval = candidate_sval[i];
            }
        }
        double kalman_gain = current_var / (current_var + sensed_var);
        double new_val = kalman_gain * new_sval + (1 - kalman_gain) * current_val;
        double new_var = kalman_gain * sensed_var + (1 - kalman_gain) * current_var;
        if (Double.isNaN(new_val) || Double.isNaN(new_var)) {
            double[] new_1 = { current_val, current_var };
            return new_1;
        } else {
            double[] new_1 = { new_val, new_var };
            return new_1;
        }
    }
    // ================
    // GET SET VALUES
    // ================

    // pos
    public double[] getPosVal() {
        return this.Phy.Val.pos;
    }

    public double getPosVar() {
        return this.Phy.Var.pos;
    }

    public void setPos(double[] val, double var) {
        this.Phy.Val.pos = val;
        this.Phy.Var.pos = var;
    }

    // vel
    public double[] getVelVal() {
        return this.Phy.Val.vel;
    }

    public double getVelVar() {
        return this.Phy.Var.vel;
    }

    public void setVel(double[] val, double var) {
        this.Phy.Val.vel = val;
        this.Phy.Var.vel = var;
    }

    // acc
    public double[] getAccVal() {
        return this.Phy.Val.acc;
    }

    public double getAccVar() {
        return this.Phy.Var.acc;
    }

    public void setAcc(double[] val, double var) {
        this.Phy.Val.acc = val;
        this.Phy.Var.acc = var;
    }

    // heading
    public double getHeadingVal() {
        return this.Phy.Val.heading;
    }

    public double getHeadingVar() {
        return this.Phy.Var.heading;
    }

    public void setHeading(double val, double var) {
        this.Phy.Val.heading = val;
        this.Phy.Var.heading = var;
    }

    // ang_vel
    public double getAngVelVal() {
        return this.Phy.Val.ang_vel;
    }

    public double getAngVelVar() {
        return this.Phy.Var.ang_vel;
    }

    public void setAngVel(double val, double var) {
        this.Phy.Val.ang_vel = val;
        this.Phy.Var.ang_vel = var;
    }

    // ang_acc
    public double getAngAccVal() {
        return this.Phy.Val.ang_acc;
    }

    public double getAngAccVar() {
        return this.Phy.Var.ang_acc;
    }

    public void setAngAcc(double val, double var) {
        this.Phy.Val.ang_acc = val;
        this.Phy.Var.ang_acc = var;
    }

    // WHL RPM
    public double getFLRadssVal() {
        return this.Whl.Val.fl_radss;
    }

    public double getFLRadssVar() {
        return this.Whl.Var.fl_radss;
    }

    public void setFLRadss(double val, double var) {
        this.Whl.Val.fl_radss = val;
        this.Whl.Var.fl_radss = var;
    }

    public double getFRRadssVal() {
        return this.Whl.Val.fr_radss;
    }

    public double getFRRadssVar() {
        return this.Whl.Var.fr_radss;
    }

    public void setFRRadss(double val, double var) {
        this.Whl.Val.fr_radss = val;
        this.Whl.Var.fr_radss = var;
    }

    public double getBLRadssVal() {
        return this.Whl.Val.bl_radss;
    }

    public double getBLRadssVar() {
        return this.Whl.Var.bl_radss;
    }

    public void setBLRadss(double val, double var) {
        this.Whl.Val.bl_radss = val;
        this.Whl.Var.bl_radss = var;
    }

    public double getBRRadssVal() {
        return this.Whl.Val.br_radss;
    }

    public double getBRRadssVar() {
        return this.Whl.Var.br_radss;
    }

    public void setBRRadss(double val, double var) {
        this.Whl.Val.br_radss = val;
        this.Whl.Var.br_radss = var;
    }

    // Whl Odo Pos
    public double[] getWhlOPosVal() {
        return this.Whl.Val.whl_o_pos;
    }

    public double getWhlOPosVar() {
        return this.Whl.Var.whl_o_pos;
    }

    public void setWhlOPos(double[] val, double var) {
        this.Whl.Val.whl_o_pos = val;
        this.Whl.Var.whl_o_pos = var;
    }

    // Whl Odo Vel
    public double[] getWhlOVelVal() {
        return this.Whl.Val.whl_o_vel;
    }

    public double getWhlOVelVar() {
        return this.Whl.Var.whl_o_vel;
    }

    public void setWhlOVel(double[] val, double var) {
        this.Whl.Val.whl_o_vel = val;
        this.Whl.Var.whl_o_vel = var;
    }

    // Whl Odo Heading
    public double getWhlOHeadingVal() {
        return this.Whl.Val.whl_o_heading;
    }

    public double getWhlOHeadingVar() {
        return this.Whl.Var.whl_o_heading;
    }

    public void setWhlOHeading(double val, double var) {
        this.Whl.Val.whl_o_heading = val;
        this.Whl.Var.whl_o_heading = var;
    }

    // Whl Odo Ang Vel
    public double getWhlOAngVelVal() {
        return this.Whl.Val.whl_o_angvel;
    }

    public double getWhlOAngVelVar() {
        return this.Whl.Var.whl_o_angvel;
    }

    public void setWhlOAngVel(double val, double var) {
        this.Whl.Val.whl_o_angvel = val;
        this.Whl.Var.whl_o_angvel = var;
    }

    // Whl Odo Torque
    public double getFLTVal() {
        return this.Whl.Val.fl_t;
    }

    public double getFLTVar() {
        return this.Whl.Var.fl_t;
    }

    public void setFLT(double val, double var) {
        this.Whl.Val.fl_t = val;
        this.Whl.Var.fl_t = var;
    }

    public double getFRTVal() {
        return this.Whl.Val.fr_t;
    }

    public double getFRTVar() {
        return this.Whl.Var.fr_t;
    }

    public void setFRT(double val, double var) {
        this.Whl.Val.fr_t = val;
        this.Whl.Var.fr_t = var;
    }

    public double getBLTVal() {
        return this.Whl.Val.bl_t;
    }

    public double getBLTVar() {
        return this.Whl.Var.bl_t;
    }

    public void setBLT(double val, double var) {
        this.Whl.Val.bl_t = val;
        this.Whl.Var.bl_t = var;
    }

    public double getBRTVal() {
        return this.Whl.Val.br_t;
    }

    public double getBRTVar() {
        return this.Whl.Var.br_t;
    }

    public void setBRT(double val, double var) {
        this.Whl.Val.br_t = val;
        this.Whl.Var.br_t = var;
    }

    // get storage
    public double getSLIDARVal() {
        return this.Sto.val.s_lidar_dist;
    }

    public double getSLIDARVar() {
        return this.Sto.var.s_lidar_dist;
    }

    public void setSLIDAR(double val, double var) {
        this.Sto.val.s_lidar_dist = val;
        this.Sto.var.s_lidar_dist = var;
    }

    public double[] getShooterVal() {
        return this.Sho.val.shooter_radss;
    }

    public double getShooterVar() {
        return this.Sho.var.shooter_radss;
    }

    public void setShooter(double[] shooter_radss, double var) {
        this.Sho.val.shooter_radss[0] = shooter_radss[0];
        this.Sho.val.shooter_radss[1] = shooter_radss[1];
        this.Sho.var.shooter_radss = var;
    }

    public double getStorage2Val() {
        return this.Sho.val.storage_radss;
    }

    public double getStorage2Var() {
        return this.Sho.var.storage_radss;
    }

    public void setStorage2(double storage_radss, double var) {
        this.Sho.val.storage_radss = storage_radss;
        this.Sho.var.storage_radss = var;
    }

    public double getBeam0Val() {
        return this.b.val.beam_0;
    }

    public double getBeam0Var() {
        return this.b.var.beam_0;
    }

    public void setBeam0(double val, double var) {
        this.b.val.beam_0 = val;
        this.b.var.beam_0 = var;
    }

    public double getBeam0_5Val() {
        return this.b.val.beam_0_5;
    }

    public double getBeam0_5Var() {
        return this.b.var.beam_0_5;
    }

    public void setBeam0_5(double val, double var) {
        this.b.val.beam_0_5 = val;
        this.b.var.beam_0_5 = var;
    }

    public double getBeam1Val() {

        return this.b.val.beam_1;
    }

    public double getBeam1Var() {
        return this.b.var.beam_1;
    }

    public void setBeam1(double val, double var) {
        this.b.val.beam_1 = val;
    }

    public double[] getLimeAVal() {
        return this.l.val.angles;
    }

    public double getLimeAVar() {
        return this.l.var.angles;
    }

    public void setLimeA(double[] val, double var) {
        this.l.val.angles = val;
        this.l.var.angles = var;
    }

    public double getLimeVVal() {
        return this.l.val.valid;
    }

    public double getLimeVVar() {
        return this.l.var.valid;
    }

    public void setLimeV(double val, double var) {
        this.l.val.valid = val;
        this.l.var.valid = var;
    }
}