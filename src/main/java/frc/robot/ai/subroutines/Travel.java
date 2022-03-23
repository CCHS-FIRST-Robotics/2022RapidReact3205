package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Travel {

    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;

    double v_max;
    double a_max;
    FwdController v_contr;

    double angvel_max;

    public double[] tpos = { 0, 0 };
    double thead = 0;

    double[] ipos = { 0, 0 };

    public void initPID() {
        this.fl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }

    double getTheta(MainState main_state) {
        double[] point_vec = SimpleMat.projectHeading(this.thead, 1);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double pwr_cmd = SimpleMat.vecsAngle2(unit_h_vec, point_vec);
        SmartDashboard.putNumberArray("Travel/point_vec", point_vec);
        SmartDashboard.putNumberArray("Travel/unit_h_vec", unit_h_vec);
        return pwr_cmd;
    }

    double getAdist(MainState state) {
        double theta = getTheta(state);
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        double adist;
        if (theta == 0) {
            adist = tdist;
        } else {
            adist = theta * tdist / (2 * Math.sin(theta / 2));
        }
        return adist;
    }

    public Command trajectory(MainState state, boolean override_adist, double nadist) {

        double theta = getTheta(state);
        //double theta = (this.thead - state.getHeadingVal());
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        double adist;
        if (theta == 0) {
            adist = tdist;
        } else {
            adist = theta * tdist / (2 * Math.sin(theta / 2));
        }
        double[] direction_vec = SimpleMat.subtract(this.tpos, state.getPosVal());
        direction_vec = SimpleMat.unitVec(direction_vec);
        direction_vec = SimpleMat.rot2d(direction_vec, -0.5 * theta);

        double current_vel = SimpleMat.dot(direction_vec, state.getVelVal());
        double target_v = this.v_contr.update(current_vel, adist);
        SmartDashboard.putNumber("Travel/target_v", target_v);
        SmartDashboard.putNumber("Travel/adist", adist);
        SmartDashboard.putNumber("Travel/adist", a_max);
        if (override_adist) {
            target_v = this.v_contr.update(current_vel, nadist);
        }

        // double current_v_max = Math.abs(this.angvel_max * tdist / (2 * Math.sin(theta
        // / 2)));
        // target_v = Math.min(target_v, current_v_max);

        double target_ang_vel = 2 * (target_v / tdist) * Math.sin(theta / 2);
        double[] local_vel = SimpleMat.rot2d(direction_vec, -1 * state.getHeadingVal());
        local_vel = SimpleMat.unitVec(local_vel);
        local_vel = SimpleMat.scaleVec(local_vel, target_v);

        double[] whl_array = MecanumIK.mecanumIK(local_vel, target_ang_vel);

        double flr = this.fl.updateRaw(whl_array[0], state.getFLRadssVal());
        double frr = this.fr.updateRaw(whl_array[1], state.getFRRadssVal());
        double blr = this.bl.updateRaw(whl_array[2], state.getBLRadssVal());
        double brr = this.br.updateRaw(whl_array[3], state.getBRRadssVal());

        SmartDashboard.putNumberArray("Travel/Whl Radss", whl_array);
        SmartDashboard.putNumber("Travel/target ang vel", target_ang_vel);
        SmartDashboard.putNumber("Travel/theta", theta);
        SmartDashboard.putNumber("Travel/thead", thead);
        SmartDashboard.putNumber("Travel/chead", state.getHeadingVal());
        SmartDashboard.putNumberArray("Travel/Direction Vec", direction_vec);
        SmartDashboard.putNumberArray("Travel/local_vel", local_vel);
        SmartDashboard.putNumber("Travel/flr", flr);
        SmartDashboard.putNumber("Travel/flr_actual", state.getFLRadssVal());
        SmartDashboard.putNumber("Travel/flr Integral", fl.integral);
        SmartDashboard.putNumber("Travel/flr P", fl.k_p);

        // return new Command(0, 0, 0, 0);
        double[] ocmd = { flr, frr, blr, brr, 0, 0, 0, 0, 0, 0, 0 };
        return new Command(ocmd);
    }
}