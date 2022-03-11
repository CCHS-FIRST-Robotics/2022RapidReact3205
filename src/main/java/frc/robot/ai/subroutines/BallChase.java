package frc.robot.ai.subroutines;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.helper.DPID;
import frc.robot.helper.MecanumIK;
import frc.robot.helper.SimpleMat;
import frc.robot.map.Ball;
import frc.robot.map.Map;
import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallChase {
    int ball_index;
    double heading;
    double final_angle;
    double[] pos = { 0, 0 };

    double[] ball_pos = { 0, 0 };
    double[] ball_vel = { 0, 0 };

    double ang_dis;

    double direction_turning;

    double max_vel;

    double vel_angle;

    double[] result = { 0, 0, 0, 0 };

    double max_ang_vel;

    double time;

    boolean NOSOL = false;

    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;

    public BallChase(double max_prop, int ball_index) {
        this.ball_index = ball_index;
        this.max_vel = 1 * max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
                / 60;
        this.max_ang_vel = 1 * max_vel / (Constants.ROBOT_WIDTH * 0.5);
    }

    public void initPID() {
        this.fl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }

    public double[] predict() {

        heading = SimpleMat.angleRectifier(heading);

        // calculate the desired final angle

        // it can be in two directions

        // - perpendicular to ball in either direction -

        // pick the optimal direction of the two

        final_angle = SimpleMat.vec2theta(ball_vel) + Math.PI / 2;

        if (Math.abs(SimpleMat.angleRectifier(final_angle - heading)) > Math.PI / 2) {

            final_angle += Math.PI;

        }

        final_angle = SimpleMat.angleRectifier(final_angle);

        // find direction robot will turn

        ang_dis = SimpleMat.angleRectifier(final_angle - heading);

        if (ang_dis >= 0) {

            direction_turning = 1;

        } else {

            direction_turning = -1;

        }

        double pot_vel_mag = angularConstraint();

        if (pot_vel_mag < max_vel) {

            // System.out.println("1");

            // printPath(maxAngVel * directionTurning, potentialVelMag, velAngle, time);

            result[0] = -1 * pot_vel_mag * Math.sin(vel_angle);

            result[1] = pot_vel_mag * Math.cos(vel_angle);

            result[2] = max_ang_vel * direction_turning;

            // for(double d : result) System.out.println(d);

            return result;

        }

        // System.out.println("2");

        double pot_ang_vel = velocityConstraint();

        vel_angle = findVelAngle(pot_ang_vel, max_vel);

        time = ang_dis / pot_ang_vel;

        // printPath(potentialAngVel, maxVel, velAngle, time);

        if (NOSOL) {

            result[3] = -1;

            return result;

        } else {

            result[0] = -1 * pot_vel_mag * Math.sin(vel_angle);

            result[1] = pot_vel_mag * Math.cos(vel_angle);

            result[2] = max_ang_vel * direction_turning;

            // for(double d : result) System.out.println(d);

            return result;

        }

    }

    // return the velocity if angular velocity is the constraint

    public double angularConstraint() {

        double ang_vel = max_ang_vel * direction_turning;

        double t = ang_dis / ang_vel;

        time = t;

        double[] dis = { -pos[0] + ball_pos[0] + t * ball_vel[0], -pos[1] + ball_pos[1] + t * ball_vel[1] };

        double dist = SimpleMat.mag(dis);

        double r = Math.abs((dist / 2) / Math.sin(ang_dis / 2));

        double velMag = Math.abs(r * ang_vel);

        double[] dis2 = { direction_turning * r * (Math.cos(ang_dis) - 1), r * Math.sin(Math.abs(ang_dis)) };

        vel_angle = SimpleMat.angleRectifier(SimpleMat.vec2theta(dis) - SimpleMat.vec2theta(dis2));

        return velMag;

    }

    // return the angular velocity if velocity is the constraint

    public double velocityConstraint() {

        double c1 = max_vel * max_vel * pow((2 * Math.sin(ang_dis / 2)), 2);

        double[] bpp = SimpleMat.subtract(ball_pos, pos);

        double c2 = pow(SimpleMat.mag(bpp), 2);

        double c3 = 2 * ang_dis

                * ((-pos[0] + ball_pos[0]) * ball_vel[0]

                        + (-pos[1] + ball_pos[1]) * ball_vel[1]);

        double c4 = pow(ang_dis, 2) * pow(SimpleMat.mag(ball_vel), 2);

        double a = c2;

        double b = c3;

        double c = c4 - c1;

        double root1 = 0;

        double root2 = 0;

        if (pow(b, 2) - 4 * a * c < 0) {

            NOSOL = true;

            return 0;

        }

        root1 = (-b + Math.sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);

        root2 = (-b - Math.sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);

        // System.out.println(root1 + " " + root2);

        boolean root1Works = true;

        boolean root2Works = true;

        if (root1 * direction_turning < 0)
            root1Works = false;

        if (Math.abs(root1) > max_ang_vel)
            root1Works = false;

        if (root2 * direction_turning < 0)
            root2Works = false;

        if (Math.abs(root2) > max_ang_vel)
            root2Works = false;

        if (!root1Works && !root2Works) {

            NOSOL = true;

            return 0;

        }

        if (!root1Works)
            return root2;

        if (!root2Works)
            return root1;

        if (direction_turning < 0)
            return Math.min(root1, root2);

        return Math.max(root1, root2);

    }

    public double findVelAngle(double angVel, double velMag) {

        double t = ang_dis / angVel;

        double[] dis = { -pos[0] + ball_pos[0] + t * ball_vel[0], -pos[1] + ball_pos[1] + t * ball_vel[1] };

        double dist = SimpleMat.mag(dis);

        double r = Math.abs((dist / 2) / Math.sin(ang_dis / 2));

        double[] dis2 = { direction_turning * r * (Math.cos(ang_dis) - 1), r * Math.sin(Math.abs(ang_dis)) };

        return SimpleMat.angleRectifier(SimpleMat.vec2theta(dis) - SimpleMat.vec2theta(dis2));

    }

    public double pow(double a, double b) {

        return Math.pow(a, b);

    }

    public Command update(MainState state, Map map) {
        this.pos[0] = state.getPosVal()[0];
        this.pos[1] = state.getPosVal()[1];

        this.heading = state.getHeadingVal();

        Ball ball = map.balls[this.ball_index];

        this.ball_pos[0] = ball.ball_pos[0];
        this.ball_pos[1] = ball.ball_pos[1];

        this.ball_vel[0] = ball.ball_vel[0];
        this.ball_vel[1] = ball.ball_vel[1];

        double[] des = predict();
        double[] outputs = Constants.DEFAULT_CMD;
        SmartDashboard.putNumberArray("Ball Chase/des", des);
        SmartDashboard.putNumberArray("Ball Chase/ball pos", ball_pos);
        SmartDashboard.putNumberArray("Ball Chase/ball vel", ball_vel);
        if (this.NOSOL) {
            outputs = Constants.DEFAULT_CMD;
        } else {
            double[] whl_array = MecanumIK.mecanumIK(new double[] { des[0], des[1] }, des[2]);
            double flr = this.fl.update(whl_array[0] - state.getFLRadssVal());
            double frr = this.fr.update(whl_array[1] - state.getFRRadssVal());
            double blr = this.bl.update(whl_array[2] - state.getBLRadssVal());
            double brr = this.br.update(whl_array[3] - state.getBRRadssVal());
            // intake and storage are both 0.7
            double intake_resp = 0.7;
            double storage_resp = 0.7;
            outputs[0] = flr;
            outputs[1] = frr;
            outputs[2] = blr;
            outputs[3] = brr;
            outputs[4] = intake_resp;
            outputs[5] = storage_resp;
        }
        return new Command(outputs);
    }

    public boolean exit(MainState state, Map map) {
        Ball ball = map.balls[this.ball_index];
        if (state.getBeam0Val() == 1) {
            return true;
        }
        if (ball.state == 0) {
            return true;
        }
        return false;
    }
}
