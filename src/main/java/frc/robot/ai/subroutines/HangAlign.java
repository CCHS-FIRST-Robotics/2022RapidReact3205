package frc.robot.ai.subroutines;

import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.helper.*;
import frc.robot.state.MainState;

public class HangAlign extends Turn {
    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;

    double target_heading;

    public HangAlign( double max_prop) {
        this.max_prop = max_prop;
        this.stime = (double) System.currentTimeMillis() / 1000;

        this.angvel_max = 0.5 * max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 4 * Math.PI
                / (60 * Constants.ROBOT_WIDTH);

        double cross_tc = Math.sin(Math.PI * 0.75 - Math.atan(Constants.ROBOT_WIDTH / Constants.ROBOT_LENGTH));
        double[] p_vec = { Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH };
        double adist = SimpleMat.mag(p_vec);
        this.angacc_max = 0.5 * max_prop * adist * cross_tc * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.MOI);
        if (Constants.TEAM == 0){
            target_heading = Math.PI / 2;
        }
        else{
            target_heading = -1 * Math.PI / 2;
        }

        this.fl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }
    
    public Command update(MainState state) {
        double dtheta = SimpleMat.angleRectifier(this.target_heading - state.getHeadingVal());
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
    public void init(MainState state) {
        this.init_dtheta = SimpleMat.angleRectifier(this.target_heading - state.getHeadingVal());
        this.stime = (double) System.currentTimeMillis() / 1000;
    }

    public boolean exit(MainState state) {
        double remaining = Math.abs(SimpleMat.angleRectifier(this.target_heading - state.getHeadingVal()));
        if (remaining < Constants.ACCEPTABLE_ANGLE_ERROR) {
            return true;
        }
        double ctime = (double) System.currentTimeMillis() / 1000;
        double time_limit = this.init_dtheta / (this.angvel_max * 0.0001);
        if ((ctime - stime) > time_limit) {
            // return true;
        }
        return false;
    }
}
