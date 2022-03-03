package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FiringPosition extends Travel {
    double stime;
    double init_dist;

    public FiringPosition(MainState state, double max_prop) {

        this.stime = (double) System.currentTimeMillis() / 1000;
        this.init_dist = 10;

        this.v_max = max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
                / 60;
        this.a_max = 0.8 * max_prop * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.ROBOT_MASS);
        SmartDashboard.putNumber("vmax", this.v_max);
        SmartDashboard.putNumber("amax", this.a_max);
        this.v_contr = new FwdController(this.v_max, this.a_max);

        this.angvel_max = max_prop * Constants.ROBOT_WIDTH * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * Math.PI
                / 60;

        double[] target_pos = SimpleMat.scaleVec(state.getPosVal(), Constants.FIRING_DIST);
        double target_heading = SimpleMat.vecsAngle2(new double[] { 0, 1 }, state.getPosVal());

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

        this.stime = (double) System.currentTimeMillis() / 1000;
        this.init_dist = getAdist(state);
    }

    public boolean exit(MainState state) {
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
        double ctime = (double) System.currentTimeMillis() / 1000;
        double time_limit = this.init_dist / (this.v_max * 0.01);
        if ((ctime - stime) > time_limit) {
            // return true;
        }

        double vel_mag = SimpleMat.mag(state.getVelVal()) + SimpleMat.mag(state.getAccVal()) * Constants.MAIN_DT;
        double avel_mag = Math.abs(state.getAngVelVal()) + Math.abs(state.getAngAccVal()) * Constants.MAIN_DT;
        if (vel_mag < 0.01 && avel_mag < 0.01 && tdist < 0.15) {
            return true;
        }
        return false;
    }

    public Command update(MainState state) {
        return trajectory(state, false, 0);
    }
}
