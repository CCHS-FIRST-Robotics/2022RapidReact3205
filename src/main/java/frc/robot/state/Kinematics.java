package frc.robot.state;

import frc.robot.Constants;
import frc.robot.helper.SimpleMat;
import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class handles robot motion kinematics. Stores values and variances and
 * calculates updates over dt time.
 * 
 * @author Ludwig Tay
 */
public class Kinematics {

    public double friction = Constants.INIT_FRICTION;

    public Values Val = new Values();
    public Variances Var = new Variances();

    /**
     * Constructor for Kinematics class.
     */
    public Kinematics() {
        this.friction = Constants.INIT_FRICTION;
    }

    /**
     * predict values and variances over dt time
     * 
     * @param dt time in seconds for update
     */
    public void predict(double dt) {
        this.Val.predict(dt);
        this.Var.predict(dt);
        SmartDashboard.putNumber("Kin Heading", this.Val.heading);
        SmartDashboard.putNumberArray("Kin Pos", this.Val.pos);
        SmartDashboard.putNumberArray("Kin Acc", this.Val.acc);
    }

    /**
     * Embedded class that holds values for kinematics parameters
     * 
     * @author Ludwig Tay
     */
    class Values {
        public double[] pos = Constants.INIT_POS;
        public double[] vel = Constants.INIT_VEL;
        public double[] acc = Constants.INIT_ACC;

        public double heading = Constants.INIT_HEADING;
        public double ang_vel = Constants.INIT_ANG_VEL;
        public double ang_acc = Constants.INIT_ANG_ACC;

        /**
         * Constructor for values
         */
        public Values() {
            this.pos = Constants.INIT_POS;
            this.vel = Constants.INIT_VEL;
            this.acc = Constants.INIT_ACC;

            this.heading = Constants.INIT_HEADING;
            this.ang_vel = Constants.INIT_ANG_VEL;

        }

        private double[] TaylorDisplacement(double t) {

            double a = ang_acc;
            double w = ang_vel;
            double v = SimpleMat.mag(vel);

            /**
             * v x
             * - (v w^2 x^3)/6
             * - (a v w x^4)/8
             * - (a^2 v x^5)/40
             * + (v w^4 x^5)/120
             * + (a v w^3 x^6)/72
             **/
            double yDis = t
                    - (Math.pow(w, 2) * Math.pow(t, 3)) / 6
                    - (a * w * Math.pow(t, 4)) / 8
                    - (Math.pow(a, 2) * Math.pow(t, 5)) / 40
                    + (Math.pow(w, 4) * Math.pow(t, 5)) / 120
                    + (a * Math.pow(w, 3) * Math.pow(t, 6)) / 72;
            yDis *= v;

            /**
             * -(v w x^2)/2
             * - (a v x^3)/6
             * + (v w^3 x^4)/24
             * + (a v w^2 x^5)/20
             * + (a^2 v w x^6)/48
             * - (v w^5 x^6)/720
             **/
            double xDis = -(w * Math.pow(t, 2)) / 2
                    - (a * Math.pow(t, 3)) / 6
                    + (Math.pow(w, 3) * Math.pow(t, 4)) / 24
                    + (a * Math.pow(w, 2) * Math.pow(t, 5)) / 20
                    + (Math.pow(a, 2) * w * Math.pow(t, 6)) / 48
                    - (Math.pow(w, 5) * Math.pow(t, 6)) / 720;
            xDis *= v;

            double[] dis = { xDis, yDis };
            dis = SimpleMat.rot2d(dis, heading);

            return dis;
        }

        /**
         * Predict all kinematics values for dt time.
         * 
         * @param dt update time in seconds.
         */
        public void predict(double dt) {
            // FRICTION ACCEL
            double[] vel_unit = SimpleMat.unitVec(this.vel);
            double[] friction_a = { friction * Constants.GRAV_ACC * vel_unit[0],
                    friction * Constants.GRAV_ACC * vel_unit[1] };
            double vel_mag = SimpleMat.mag(this.vel);
            double friction_mag = SimpleMat.mag(friction_a);
            if (friction_mag * dt > vel_mag) {
                SimpleMat.scaleVec(friction_a, SimpleMat.mag(this.vel) / (dt * friction_mag));
            }
            // NEW POSITION CALCULATION WITH TAYLOR SERIES
            double[] pos1 = { this.pos[0], this.pos[1] };
            double[] taylor_dis = TaylorDisplacement(dt);
            SmartDashboard.putNumberArray("Kinematics/TaylorDis", taylor_dis);
            pos1[0] += taylor_dis[0];
            pos1[1] += taylor_dis[1];

            // POS
            this.pos[0] = this.pos[0] + this.vel[0] * dt + 0.5 * (this.acc[0] + friction_a[0]) * dt * dt;
            this.pos[1] = this.pos[1] + this.vel[1] * dt + 0.5 * (this.acc[1] + friction_a[1]) * dt * dt;

            // AVERAGE POS
            this.pos[0] = (pos1[0] + pos[0]) / 2;
            this.pos[1] = (pos1[1] + pos[1]) / 2;

            // VEL
            this.vel[0] = this.vel[0] + (this.acc[0] + friction_a[0]) * dt;
            this.vel[1] = this.vel[1] + (this.acc[1] + friction_a[1]) * dt;
            // HEADING
            double m_o_i = Constants.ROBOT_WIDTH * Constants.ROBOT_WIDTH * Constants.ROBOT_MASS * 0.125;
            double ang_fric = (this.ang_vel / (Math.abs(this.ang_vel) + 0.001)) * Constants.ROBOT_WIDTH
                    * Constants.GRAV_ACC * friction / (2 * m_o_i);

            double ang_disp = this.ang_vel * dt + 0.5 * (this.ang_acc + ang_fric) * dt * dt;
            this.heading = this.heading + ang_disp;
            this.heading = SimpleMat.angleRectifier(this.heading);
            // ANG VEL
            if (Math.abs(this.ang_acc) < Math.abs(ang_fric)) {
                if (Math.abs(this.ang_acc + ang_fric) * dt > Math.abs(this.ang_vel)) {
                    this.ang_vel = 0;
                } else {
                    this.ang_vel = this.ang_vel + (this.ang_acc + ang_fric) * dt;
                }
            } else {
                this.ang_vel = this.ang_vel + (this.ang_acc + ang_fric) * dt;
            }
            if (Double.isNaN(heading)) {
                heading = 0;
            }
            this.vel = SimpleMat.rot2d(this.vel, ang_disp);
            this.acc = SimpleMat.rot2d(this.vel, ang_disp);

        }
    }

    /**
     * Subclass that stores and predicts variances for all the kinematics parameters
     * 
     * @author Ludwig Tay
     */
    class Variances {
        public double pos = Constants.INIT_VARIANCE;
        public double vel = Constants.INIT_VARIANCE;
        public double acc = Constants.INIT_VARIANCE;

        public double heading = Constants.INIT_VARIANCE;
        public double ang_vel = Constants.INIT_VARIANCE;
        public double ang_acc = Constants.INIT_VARIANCE;

        /**
         * Constructor for Variances
         */
        public Variances() {
            this.pos = Constants.INIT_VARIANCE;
            this.vel = Constants.INIT_VARIANCE;
            this.acc = Constants.INIT_VARIANCE;

            this.heading = Constants.INIT_VARIANCE;
            this.ang_vel = Constants.INIT_VARIANCE;
            this.ang_acc = Constants.INIT_VARIANCE;

        }

        /**
         * Predict variances for dt time in the future.
         * 
         * @param dt predict future time in seconds.
         */
        public void predict(double dt) {
            this.pos = this.pos + this.vel * dt + 0.5 * this.acc * dt * dt;
            this.vel = this.vel + this.acc * dt;
            this.heading = this.heading + this.ang_vel * dt + 0.5 * this.ang_acc * dt * dt;
            this.ang_vel = this.ang_vel + this.ang_acc * dt;
        }
    }
}
