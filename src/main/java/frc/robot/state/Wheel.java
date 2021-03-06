package frc.robot.state;

import frc.robot.Constants;
import frc.robot.helper.SimpleMat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wheel {
    public Values Val = new Values();
    public Variances Var = new Variances();

    public Wheel() {
    }

    public void predict(double dt, double heading, double[] acc, double ang_vel) {
        this.Val.predict(dt, heading, acc, ang_vel);
        this.Var.predict(dt, this.Val.fl_radss, this.Val.fr_radss, this.Val.bl_radss, this.Val.br_radss);
        // SmartDashboard.putNumber("FL rpm", this.Val.fl_radss * 30 / Math.PI);
        // SmartDashboard.putNumber("FR rpm", this.Val.fr_radss * 30 / Math.PI);
        // SmartDashboard.putNumber("BL rpm", this.Val.bl_radss * 30 / Math.PI);
        // SmartDashboard.putNumber("BR rpm", this.Val.br_radss * 30 / Math.PI);
        // SmartDashboard.putNumberArray("Odo Pos", this.Val.whl_o_pos);
        // SmartDashboard.putNumberArray("Odo Vel", this.Val.whl_o_vel);
        // SmartDashboard.putNumber("Odo Heading", this.Val.whl_o_heading);
    }

    class Values {
        public double fl_radss;
        public double fr_radss;
        public double bl_radss;
        public double br_radss;

        public double fl_t;
        public double fr_t;
        public double bl_t;
        public double br_t;

        public double[] whl_o_pos = { 0, 0 };
        public double whl_o_heading;

        public double[] whl_o_vel = { 0, 0 };
        public double whl_o_angvel;

        public Values() {
            this.fl_radss = Constants.INIT_WHL_RPM;
            this.fr_radss = Constants.INIT_WHL_RPM;
            this.bl_radss = Constants.INIT_WHL_RPM;
            this.br_radss = Constants.INIT_WHL_RPM;

            this.fl_t = 0;
            this.fr_t = 0;
            this.bl_t = 0;
            this.br_t = 0;

            this.whl_o_pos[0] = Constants.START_POS[0];
            this.whl_o_pos[1] = Constants.START_POS[1];
            this.whl_o_heading = 0;

            this.whl_o_vel[0] = 0;
            this.whl_o_vel[1] = 0;

            this.whl_o_angvel = 0;
        }

        double[] computeVels(double[] acc, double ang_vel) {
            double[] wt = { this.fl_t, this.fr_t, this.bl_t, this.br_t };
            double q = 4 * Constants.WHEEL_MOI / (Constants.WHEEL_RADIUS * Constants.WHEEL_RADIUS);
            double qvyw = -1 * (Constants.ROBOT_MASS + q) * acc[0]
                    - (wt[1] + wt[2] - wt[0] - wt[3]) / Constants.WHEEL_RADIUS;
            double qvxw = -1 * (Constants.ROBOT_MASS + q) * acc[1]
                    + (wt[1] + wt[2] + wt[0] + wt[3]) / Constants.WHEEL_RADIUS;
            double[] vel = { qvxw / (q * ang_vel), qvyw / (q * ang_vel) };
            return vel;
        }

        public void predict(double dt, double heading, double[] acc, double ang_vel) {
            // Local vel transform
            double r4 = Constants.WHEEL_RADIUS / 4;
            double turn_denom = 2 / (Constants.ROBOT_LENGTH + Constants.ROBOT_WIDTH);

            double vfwd = r4 * (this.fl_radss + this.fr_radss + this.bl_radss + this.br_radss);
            double vside = r4 * (this.fl_radss - this.fr_radss - this.bl_radss + this.br_radss);
            double angvel = r4 * turn_denom * (this.fl_radss * -1 + this.fr_radss - this.bl_radss + this.br_radss);
            double theta = angvel * dt;

            // experimental computeVels
            double[] e_vel = { vside, vfwd };
            if (ang_vel != 0) {
                double[] local_acc = SimpleMat.rot2d(acc, heading * -1);
                double[] ec_vel = computeVels(local_acc, ang_vel);
                e_vel[0] = ec_vel[0];
                e_vel[1] = ec_vel[1];
                // SmartDashboard.putNumberArray("Odo experimental vel", e_vel);
            }
            // Local pos transform

            double[] xyl_disp = { 0, 0 };
            double tl_disp = 0;
            // SmartDashboard.putNumber("odo angvel", angvel);
            if (Math.abs(angvel) == 0) {
                xyl_disp[0] = vside * dt;
                xyl_disp[1] = vfwd * dt;
                tl_disp = 0;
            } else {
                double[] r_vec = { vside / Math.abs(angvel), vfwd / Math.abs(angvel) };
                double[] A = { 0, 0 };
                if (angvel < 0) {
                    A[0] = 1 * r_vec[1];
                    A[1] = -1 * r_vec[0];
                } else {
                    A[0] = -1 * r_vec[1];
                    A[1] = 1 * r_vec[0];
                }
                xyl_disp = SimpleMat.rot2d(SimpleMat.scaleVec(A, -1), theta);
                xyl_disp = SimpleMat.add(A, xyl_disp);
                // xyl_disp[0] = (1 - Math.cos(theta)) * A[0] + Math.sin(theta) * A[1];
                // xyl_disp[1] = (1 - Math.cos(theta)) * A[1] + Math.sin(theta) * A[0];
                // SimpleMat.scaleVec(xyl_disp, dt);
                tl_disp = theta;
            }
            theta = SimpleMat.angleRectifier(theta);
            double[] local_vel = { xyl_disp[0] / dt, xyl_disp[1] / dt };
            this.whl_o_angvel = angvel;
            // offset xyl_disp and local vel
            // SmartDashboard.putNumberArray("Odo xyldisp", local_vel);
            // SmartDashboard.putNumber("Odo given h", heading);
            this.whl_o_vel = SimpleMat.rot2d(local_vel, heading);
            this.whl_o_pos = SimpleMat.add(this.whl_o_pos, SimpleMat.rot2d(xyl_disp, heading));
            this.whl_o_heading = SimpleMat.angleRectifier(this.whl_o_heading + theta);
        }
    }

    class Variances {
        public double fl_radss;
        public double fr_radss;
        public double bl_radss;
        public double br_radss;

        public double fl_t;
        public double fr_t;
        public double bl_t;
        public double br_t;

        public double whl_o_pos;
        public double whl_o_heading;

        public double whl_o_vel;
        public double whl_o_angvel;

        public Variances() {
            this.fl_radss = Constants.VAR_RAD_VAR;
            this.fr_radss = Constants.VAR_RAD_VAR;
            this.bl_radss = Constants.VAR_RAD_VAR;
            this.br_radss = Constants.VAR_RAD_VAR;

            this.fl_t = Constants.INIT_VARIANCE;
            this.fr_t = Constants.INIT_VARIANCE;
            this.bl_t = Constants.INIT_VARIANCE;
            this.br_t = Constants.INIT_VARIANCE;

            this.whl_o_pos = Constants.INIT_VARIANCE;
            this.whl_o_heading = Constants.INIT_VARIANCE;

            this.whl_o_vel = Constants.INIT_VARIANCE;
            this.whl_o_angvel = Constants.INIT_VARIANCE;
        }

        public void predict(double dt, double fl, double fr, double bl, double br) {
            double a_pwr_prop = (Math.abs(fl) * this.fl_radss + Math.abs(fr) * this.fr_radss
                    + Math.abs(bl) * this.bl_radss + Math.abs(br) * this.br_radss);
            this.whl_o_vel = Constants.WHEEL_RADIUS * a_pwr_prop / 4;
            this.whl_o_angvel = 10 * Constants.WHEEL_RADIUS
                    * (2 / (Constants.ROBOT_LENGTH + Constants.ROBOT_WIDTH))
                    * a_pwr_prop / 4;
            this.whl_o_pos = this.whl_o_pos + this.whl_o_vel * dt;
            this.whl_o_heading = this.whl_o_heading + this.whl_o_angvel * dt;
        }
    }
}
