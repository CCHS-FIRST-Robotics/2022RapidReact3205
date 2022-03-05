package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Travel {

    MPID fl;
    MPID fr;
    MPID bl;
    MPID br;

    double v_max;
    double a_max;
    FwdController v_contr;

    double angvel_max;

    public double[] tpos = { 0, 0 };
    double thead = 0;

    double[] ipos = { 0, 0 };

    double getAdist(MainState state) {
        double theta = this.thead - state.getHeadingVal();
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
        double theta = this.thead - state.getHeadingVal();
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
        if (override_adist) {
            target_v = this.v_contr.update(current_vel, nadist);
        }

        // double current_v_max = Math.abs(this.angvel_max * tdist / (2 * Math.sin(theta
        // / 2)));
        // target_v = Math.min(target_v, current_v_max);

        double target_ang_vel = 2 * (target_v / tdist) * Math.sin(theta / 2);
        double[] local_vel = SimpleMat.rot2d(direction_vec, -1 * state.getHeadingVal());
        local_vel = SimpleMat.scaleVec(local_vel, target_v);

        double[] whl_array = MecanumIK.mecanumIK(local_vel, target_ang_vel);

        double flr = this.fl.update(whl_array[0] - state.getFLRadssVal());
        double frr = this.fr.update(whl_array[1] - state.getFRRadssVal());
        double blr = this.bl.update(whl_array[2] - state.getBLRadssVal());
        double brr = this.br.update(whl_array[3] - state.getBRRadssVal());

        SmartDashboard.putNumberArray("Travel/Whl Radss", whl_array);
        SmartDashboard.putNumber("Travel/target ang vel", target_ang_vel);
        SmartDashboard.putNumber("Travel/theta", theta);
        SmartDashboard.putNumberArray("Travel/Direction Vec", direction_vec);

        // return new Command(0, 0, 0, 0);
        double[] ocmd = { flr, frr, blr, brr, 0, 0 };
        return new Command(ocmd);
    }
}