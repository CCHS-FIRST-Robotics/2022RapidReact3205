package frc.robot.state;

import frc.robot.Constants;
import frc.robot.helper.SimpleMat;

public class Wheel {
    public Values Val = new Values();
    public Variances Var = new Variances();

    public Wheel() {

    }

    class Values {
        public double fl_radss;
        public double fr_radss;
        public double bl_radss;
        public double br_radss;

        public double[] whl_o_pos;
        public double whl_o_heading;

        public double[] whl_o_vel;
        public double whl_o_angvel;

        public Values() {
            this.fl_radss = Constants.INIT_WHL_RPM;
            this.fr_radss = Constants.INIT_WHL_RPM;
            this.bl_radss = Constants.INIT_WHL_RPM;
            this.br_radss = Constants.INIT_WHL_RPM;

            this.whl_o_pos[0] = 0;
            this.whl_o_pos[1] = 0;
            this.whl_o_heading = 0;

            this.whl_o_vel[0] = 0;
            this.whl_o_vel[1] = 0;

            this.whl_o_angvel = 0;
        }

        public void predict(double dt, double heading) {
            // Local vel transform
            double r4 = Constants.WHEEL_RADIUS / 4;
            double turn_denom = 1 / (Constants.ROBOT_LENGTH + Constants.ROBOT_WIDTH);

            double vfwd = r4 * (this.fl_radss + this.fr_radss + this.bl_radss + this.br_radss);
            double vside = r4 * (this.fl_radss - this.fr_radss - this.bl_radss + this.br_radss);
            double angvel = r4 * turn_denom * (this.fl_radss * -1 + this.fr_radss - this.bl_radss + this.br_radss);
            double theta = angvel * dt;
            // Local pos transform
            double[] xyl_disp = { 0, 0 };
            double tl_disp = 0;
            if (angvel == 0) {
                xyl_disp[0] = vside * dt;
                xyl_disp[1] = vfwd * dt;
                tl_disp = 0;
            } else {
                double[] r_vec = { vside / angvel, vfwd / angvel };
                double[] A = { 0, 0 };
                if (angvel < 0) {
                    A[0] = r_vec[1];
                    A[1] = -1 * r_vec[0];
                } else {
                    A[0] = -1 * r_vec[1];
                    A[1] = r_vec[0];
                }
                xyl_disp[0] = (1 - Math.cos(theta)) * A[0] + Math.sin(theta) * A[1];
                xyl_disp[1] = (1 - Math.cos(theta)) * A[1] + Math.sin(theta) * A[0];
                tl_disp = theta;
            }
            double[] local_vel = { xyl_disp[0] / dt, xyl_disp[1] / dt };
            this.whl_o_angvel = angvel;
            // offset xyl_disp and local vel
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

    }
}
