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
            // POS
            this.pos[0] = this.pos[0] + this.vel[0] * dt + 0.5 * (this.acc[0] + friction_a[0]) * dt * dt;
            this.pos[1] = this.pos[1] + this.vel[1] * dt + 0.5 * (this.acc[1] + friction_a[1]) * dt * dt;
            // VEL
            this.vel[0] = this.vel[0] + (this.acc[0] + friction_a[0]) * dt;
            this.vel[1] = this.vel[1] + (this.acc[1] + friction_a[1]) * dt;
            // HEADING
            double m_o_i = Constants.ROBOT_WIDTH * Constants.ROBOT_WIDTH * Constants.ROBOT_MASS * 0.125;
            double ang_fric = (this.ang_vel / (Math.abs(this.ang_vel) + 0.001)) * Constants.ROBOT_WIDTH
                    * Constants.GRAV_ACC * friction / (2 * m_o_i);

            this.heading = this.heading + this.ang_vel * dt + 0.5 * (this.ang_acc + ang_fric) * dt * dt;
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
