package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Travel {

    PID fl;
    PID fr;
    PID bl;
    PID br;

    double v_max;
    double a_max;
    FwdController v_contr;

    double angvel_max;

    double[] tpos = { 0, 0 };
    double thead = 0;

    double[] ipos = { 0, 0 };

    public Travel(double[] target_pos, double target_heading, double max_prop) {
        
        this.v_max = max_prop * (Math.sqrt(2)/2) * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI/60;
        this.a_max = max_prop * 2 * Math.sqrt(2) * Constants.MOTOR_MAX_TORQUE / (Constants.WHEEL_RADIUS * Constants.ROBOT_MASS);
        this.v_contr = new FwdController(this.v_max, this.a_max);
        
        this.angvel_max = Constants.ROBOT_WIDTH * v_max/2;

        this.tpos[0] = target_pos[0];
        this.tpos[1] = target_pos[1];

        this.thead = SimpleMat.angleRectifier(target_heading);

        this.fl = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }

    public void init(MainState state) {
        this.ipos[0] = state.getPosVal()[0];
        this.ipos[1] = state.getPosVal()[1];
    }

    public boolean exit(MainState state){
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        if (tdist < 0.1){
            return true;
        }
        double[] tdiff = SimpleMat.subtract(state.getPosVal(), this.tpos);
        tdiff = SimpleMat.unitVec(tdiff);
        double[] sdiff = SimpleMat.subtract(this.ipos, this.tpos);
        sdiff = SimpleMat.unitVec(sdiff);
        if (SimpleMat.dot(tdiff,sdiff) < Math.cos(Math.PI * 0.25)){
            if (tdist < 0.5){
                return true;
            }
        }
        return false;
    }

    public Command update(MainState state) {
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
        
        double current_v_max = Math.abs(this.angvel_max * tdist / (2 * Math.sin(theta/2)));
        target_v = Math.min(target_v, current_v_max);

        double target_ang_vel = 2 * (target_v / tdist) * Math.sin(theta/2);
        double[] local_vel = SimpleMat.rot2d(direction_vec, -1 * state.getHeadingVal());
        local_vel = SimpleMat.scaleVec(local_vel, target_v);
        
        double[] whl_array = MecanumIK.mecanumIK(local_vel, target_ang_vel);
        
        double flr = this.fl.update(whl_array[0] - state.getFLRadssVal());
        double frr = this.fr.update(whl_array[1] - state.getFRRadssVal());
        double blr = this.bl.update(whl_array[2] - state.getBLRadssVal());
        double brr = this.br.update(whl_array[3] - state.getBRRadssVal());

        SmartDashboard.putNumberArray("Travel/Whl Radss", whl_array);
        SmartDashboard.putNumber("Travel/Desired Target V", target_v);
        SmartDashboard.putNumberArray("Travel/Direction Vec", direction_vec);

        return new Command(0, 0, 0, 0);
    }
}
