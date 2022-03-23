package frc.robot.ai.subroutines;

import frc.robot.helper.*;
import frc.robot.state.MainState;
import frc.robot.map.*;
import frc.robot.Constants;
import frc.robot.commands.Command;

public class PointAtBall extends Turn{

    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;

    int ball_index;

    public PointAtBall(MainState state, Map map, int ball_chase){
        ball_index = ball_chase;
        if (ball_chase == -1) {
            ball_index = GetBall.getClosestBall(state, map);
        }
        this.tpos[0] = map.balls[ball_index].pos[0];
        this.tpos[1] = map.balls[ball_index].pos[1];
        this.max_prop = max_prop;
        this.stime = (double) System.currentTimeMillis() / 1000;

        this.angvel_max = max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 4 * Math.PI
                / (60 * Constants.ROBOT_WIDTH);

        double cross_tc = Math.sin(Math.PI * 0.75 - Math.atan(Constants.ROBOT_WIDTH / Constants.ROBOT_LENGTH));
        double[] p_vec = { Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH };
        double adist = SimpleMat.mag(p_vec);
        this.angacc_max = 0.5 * max_prop * adist * cross_tc * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.MOI);

        this.velcontr = new FwdController(this.angvel_max, this.angacc_max);

        this.fl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);

    }

    public void init(MainState main_state) {
        this.init_dtheta = getTheta(main_state);
        this.stime = (double) System.currentTimeMillis() / 1000;
    }

    public boolean exit(MainState main_state, Map map) {
        double remaining = Math.abs(getTheta(main_state));
        if (remaining < Constants.ACCEPTABLE_ANGLE_ERROR) {
            return true;
        }
        if (map.balls[ball_index].state == 0){
            return true;
        }
        double ctime = (double) System.currentTimeMillis() / 1000;
        double time_limit = this.init_dtheta / (this.angvel_max * 0.0001);
        if ((ctime - stime) > time_limit) {
            // return true;
        }
        return false;
    }

    public Command update(MainState state, Map map) {
        this.tpos[0] = map.balls[ball_index].pos[0];
        this.tpos[1] = map.balls[ball_index].pos[1];
        double dtheta = getTheta(state);
        // double target_avel = this.velcontr.update(state.getAngVelVal(), dtheta);
        double target_avel = Math.max(Math.min(this.turn.update(dtheta),
                this.angvel_max), this.angvel_max * -1);

        double[] vel = { 0, 0 };
        double[] whl_array = MecanumIK.mecanumIK(vel, target_avel);


        double flr = this.fl.updateRaw(whl_array[0], state.getFLRadssVal());
        double frr = this.fr.updateRaw(whl_array[1], state.getFRRadssVal());
        double blr = this.bl.updateRaw(whl_array[2], state.getBLRadssVal());
        double brr = this.br.updateRaw(whl_array[3], state.getBRRadssVal());


        double[] ocmd = { flr, frr, blr, brr, 0, 0, 0, 0, 0, 0, 0 };
        return new Command(ocmd);
    }
}
