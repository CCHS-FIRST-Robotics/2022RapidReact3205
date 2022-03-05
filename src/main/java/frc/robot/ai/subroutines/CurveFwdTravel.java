package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.helper.*;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Command;
import frc.robot.Constants;

public class CurveFwdTravel extends Travel {
    ArrayList<double[]> point_list;
    public double[] l_start_pos = { 0, 0 };
    public double[] lsp2 = {0,0};
    public double final_head;

    public CurveFwdTravel(ArrayList<double[]> point_list, double final_head, double max_prop) {
        this.point_list = point_list;
        this.final_head = final_head;

        this.v_max = max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
                / 60;
        this.a_max = 0.8 * max_prop * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.ROBOT_MASS);
        SmartDashboard.putNumber("vmax", this.v_max);
        SmartDashboard.putNumber("amax", this.a_max);
        this.v_contr = new FwdController(this.v_max, this.a_max);

        this.angvel_max = max_prop * Constants.ROBOT_WIDTH * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * Math.PI
                / 60;

        this.fl = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }

    public double curveLength(MainState state) {
        // dist between current pos and plist[0]
        double dist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.point_list.get(0)));
        for (int c = 0; c < this.point_list.size() - 1; c++) {
            dist = dist + SimpleMat.mag(SimpleMat.subtract(this.point_list.get(c), this.point_list.get(c + 1)));
        }
        return dist;
    }

    public void init(MainState state) {
        //this.lsp2 = state.getPosVal();
        this.l_start_pos[0] = state.getPosVal()[0];
        this.l_start_pos[1] = state.getPosVal()[1];
        double[] heading_vec = SimpleMat.subtract(this.point_list.get(0) , state.getPosVal() );
        double heading = SimpleMat.globalAngle(heading_vec);
        this.thead = final_head;
        //this.thead = heading;
        this.tpos = this.point_list.get(0);
    }

    public boolean tripSubsection(MainState state) {
        double[] tdiff = SimpleMat.subtract(this.tpos, state.getPosVal());
        double dist = SimpleMat.mag(tdiff);
        tdiff = SimpleMat.unitVec(tdiff);
        double[] sdiff = SimpleMat.subtract(this.lsp2, this.tpos);
        sdiff = SimpleMat.unitVec(sdiff);
        double prod = SimpleMat.dot(tdiff, sdiff);
        SmartDashboard.putNumberArray("CFT/ tdiff", tdiff);
        SmartDashboard.putNumberArray("CFT/ sdiff", sdiff);
        SmartDashboard.putNumberArray("CFT/ l_start_pos", this.lsp2);
        SmartDashboard.putNumber("CFT/ trip prod", prod);
        
        if (prod > 0) {
            return true;
        }
        return false;
    }

    void getTrajParams() {
        double[] new_pos = this.point_list.get(1);
        double[] heading_vec = SimpleMat.subtract(this.point_list.get(1), this.point_list.get(0));
        double heading = SimpleMat.globalAngle(heading_vec);
        this.point_list.remove(0);
        this.tpos = new_pos;
        this.thead = final_head;
        //this.thead = heading;
    }

    public Command update(MainState state) {
        double adist = curveLength(state);
        Command cmd;
        SmartDashboard.putNumber("CFT/ adist", adist);
        if (this.point_list.size() > 1) {
            cmd = trajectory(state, true, adist);
        } else {
            cmd = trajectory(state, false, 0);
        }
        if (tripSubsection(state)) {
            //this.l_start_pos = state.getPosVal();
            this.lsp2 = state.getPosVal();
            if (this.point_list.size() > 1) {
                getTrajParams();
            } else {
                this.tpos = this.point_list.get(0);
                this.thead = this.final_head;
            }
        }
        SmartDashboard.putNumber("CFT/Steps Remaining", (double) this.point_list.size());
        SmartDashboard.putNumberArray("CFT/Current TPos", this.tpos);
        SmartDashboard.putNumber("CFT/Current THead", this.thead);
        //return new Command(Constants.DEFAULT_CMD);
        return cmd;

    }

    public boolean exit(MainState state) {
        if (this.point_list.size() > 1) {
            return false;
        }
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        if (tdist < 0.05) {
            if (Math.abs(state.getHeadingVal() - thead) < 0.1) {
                return true;
            }
        }
        double[] tdiff = SimpleMat.subtract(state.getPosVal(), this.tpos);
        tdiff = SimpleMat.unitVec(tdiff);
        double[] sdiff = SimpleMat.subtract(this.ipos, this.tpos);
        sdiff = SimpleMat.unitVec(sdiff);
        if (SimpleMat.dot(tdiff, sdiff) < Math.cos(Math.PI * 0.125)) {
            if (tdist < 0.1) {
                if (Math.abs(state.getHeadingVal() - thead) < 0.1) {
                    return true;
                }
            }
        }

        double vel_mag = SimpleMat.mag(state.getVelVal()) + SimpleMat.mag(state.getAccVal()) * Constants.MAIN_DT;
        double avel_mag = Math.abs(state.getAngVelVal()) + Math.abs(state.getAngAccVal()) * Constants.MAIN_DT;
        if (vel_mag < 0.01 && avel_mag < 0.01 && tdist < 0.15) {
            return true;
        }
        return false;
    }
}
