package frc.robot.ai.subroutines;

import frc.robot.helper.*;
import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PointAtMiddle extends Turn {

    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;

    public PointAtMiddle(MainState state, double max_prop) {
        this.tpos[0] = state.getPosVal()[0] * 2;
        this.tpos[1] = state.getPosVal()[1] * 2;
        this.max_prop = max_prop;
        this.stime = (double) System.currentTimeMillis() / 1000;

        this.angvel_max = max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 4 * Math.PI
                / (60 * Constants.ROBOT_WIDTH);

        double cross_tc = Math.sin(Math.PI * 0.75 - Math.atan(Constants.ROBOT_WIDTH / Constants.ROBOT_LENGTH));
        double[] p_vec = { Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH };
        double adist = SimpleMat.mag(p_vec);
        this.angacc_max = 0.5 * max_prop * adist * cross_tc * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.MOI);

        //SmartDashboard.putNumber("turn/angmax", this.angvel_max);
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

    public boolean exit(MainState main_state) {
        double remaining = Math.abs(getTheta(main_state));
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

    public Command update(MainState state) {
        double dtheta = getTheta(state);
        // double target_avel = this.velcontr.update(state.getAngVelVal(), dtheta);
        double target_avel = Math.max(Math.min(this.turn.update(dtheta),
                this.angvel_max), this.angvel_max * -1);

        //SmartDashboard.putNumber("PAM/dtheta", dtheta);
        //SmartDashboard.putNumberArray("PAM/TPOS", this.tpos);

        double[] vel = { 0, 0 };
        double[] whl_array = MecanumIK.mecanumIK(vel, target_avel);

        //SmartDashboard.putNumber("turn/tavel", target_avel);
        //SmartDashboard.putNumber("turn/avel_max", this.angvel_max);
        //SmartDashboard.putNumber("turn/aacc_max", this.angacc_max);
        //SmartDashboard.putNumberArray("turn/whl arr", whl_array);

        double flr = this.fl.updateRaw(whl_array[0], state.getFLRadssVal());
        double frr = this.fr.updateRaw(whl_array[1], state.getFRRadssVal());
        double blr = this.bl.updateRaw(whl_array[2], state.getBLRadssVal());
        double brr = this.br.updateRaw(whl_array[3], state.getBRRadssVal());

        //SmartDashboard.putNumber("turn/flr", flr);

        double[] ocmd = { flr, frr, blr, brr, 0, 0, 0, 0, 0, 0, 0, 0};
        return new Command(ocmd);
    }

}
