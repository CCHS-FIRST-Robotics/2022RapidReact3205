package frc.robot.ai.subroutines;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.helper.*;
import frc.robot.map.Ball;
import frc.robot.map.Map;
import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallChase {

    double vel_mag = 0;
    double ang_vel_max = 0;
    int ball_index = -1;

    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;
    PID turn;

    boolean dash_state = false;
    double dash_time = System.currentTimeMillis() / 1000;

    public BallChase(MainState state, Map map, int ball_chase, double max_prop) {
        // if ball chase = -1, scan on its own
        ball_index = ball_chase;
        if (ball_chase == -1) {
            ball_index = GetBall.getClosestBall(state, map);
        }
        this.vel_mag = 1 * max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
                / 60;
        this.ang_vel_max = 0.05 * max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 4 * Math.PI
                / (60 * Constants.ROBOT_WIDTH);
        initPID();
        this.dash_state = false;
        this.dash_time = System.currentTimeMillis() / 1000;
    }

    public void initPID() {
        this.fl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.turn = new PID(Constants.TURN_TUNING[0], Constants.TURN_TUNING[1], Constants.TURN_TUNING[2]);
    }

    public Command update(MainState state, Map map) {
        SmartDashboard.putNumber("BallChase/ball index", ball_index);
        if (this.ball_index == -1) {
            return new Command(Constants.DEFAULT_CMD);
        }
        Ball ball = map.balls[this.ball_index];
        double[] diff = SimpleMat.subtract(ball.pos, state.getPosVal());
        double dist = SimpleMat.mag(diff);
        double time = dist / this.vel_mag;
        // double[] new_pos = SimpleMat.add(ball.pos, SimpleMat.scaleVec(ball.vel,
        // time));
        double[] new_pos = ball.pos;
        double[] nadiff = SimpleMat.subtract(new_pos, state.getPosVal());
        double[] ndiff = SimpleMat.unitVec(nadiff);

        double[] unit_h_vec = SimpleMat.projectHeading(state.getHeadingVal(), 1);
        double pwr_cmd = SimpleMat.vecsAngle2(unit_h_vec, ndiff);

        // determine dash state
        if (Math.abs(pwr_cmd) < 30 * 2 * Math.PI / 360 && SimpleMat.mag(nadiff) < 0.5) {
            this.dash_state = true;
            this.dash_time = System.currentTimeMillis() / 1000;
        }

        double[] vel = SimpleMat.scaleVec(ndiff, this.vel_mag);
        vel = SimpleMat.rot2d(vel, -1 * state.getHeadingVal());
        double target_avel = pwr_cmd / time;
        if (this.dash_state) {
            vel = new double[] { 0, this.vel_mag };
            target_avel = 0;
        }
        double[] whl_array = MecanumIK.mecanumIK(vel, target_avel);
        double flr = this.fl.update(whl_array[0] - state.getFLRadssVal());
        double frr = this.fr.update(whl_array[1] - state.getFRRadssVal());
        double blr = this.bl.update(whl_array[2] - state.getBLRadssVal());
        double brr = this.br.update(whl_array[3] - state.getBRRadssVal());

        SmartDashboard.putNumber("BallChase/Vel Mag", vel_mag);
        SmartDashboard.putNumberArray("BallChase/Whl Radss", vel);
        SmartDashboard.putNumber("BallChase/target avel", target_avel);
        SmartDashboard.putNumberArray("BallChase/Ball pos", ball.pos);
        SmartDashboard.putNumberArray("BallChase/Ball npos", new_pos);
        SmartDashboard.putNumberArray("BallChase/Ball vell", ball.vel);
        // return new Command(0, 0, 0, 0);
        double[] ocmd = { flr, frr, blr, brr, 0, 0, 0, 0, 0, 0, 0 };
        SmartDashboard.putNumberArray("BallChase/OCMD", ocmd);
        return new Command(ocmd);
    }

    public boolean exit(MainState state, Map map) {
        if (this.ball_index == -1) {
            return true;
        }
        Ball ball = map.balls[this.ball_index];
        if (ball.state == 0) {
            return true;
        }
        if (state.getBeam0Val() == 1) {
            return true;
        }
        if (this.dash_state) {
            double wait_time = 0.2 + (0.5 / this.vel_mag);
            if (System.currentTimeMillis() / 1000 - this.dash_time > wait_time) {
                return true;
            }
        }
        return false;
    }
}
