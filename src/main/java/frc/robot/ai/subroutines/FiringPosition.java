package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FiringPosition extends Travel {
    double stime;
    double init_dist;
    boolean pam = false;

    public FiringPosition(MainState state, double max_prop) {

        this.pam = false;
        this.stime = (double) System.currentTimeMillis() / 1000;
        this.init_dist = 0.5;

        this.v_max = max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
                / 60;
        // this.a_max = 0.1 * max_prop * Math.sin(Math.PI / 4) * 4 *
        // Constants.MOTOR_MAX_TORQUE
        // / (Constants.WHEEL_RADIUS * Constants.ROBOT_MASS);
        this.a_max = 3;
        // SmartDashboard.putNumber("vmax", this.v_max);
        // SmartDashboard.putNumber("amax", this.a_max);
        this.v_contr = new FwdController(this.v_max, this.a_max);
        this.v_contr.jerk = this.a_max / 0.5;

        this.angvel_max = max_prop * Constants.ROBOT_WIDTH * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * Math.PI
                / 60;

        double[] target_pos = SimpleMat.scaleVec(SimpleMat.unitVec(state.getPosVal()), Constants.FIRING_DIST);
        double target_heading = SimpleMat.vecsAngle2(new double[] { 0, 1 }, state.getPosVal());

        this.tpos[0] = target_pos[0];
        this.tpos[1] = target_pos[1];

        this.thead = SimpleMat.angleRectifier(target_heading);

        initPID();
    }

    public void init(MainState state) {
        this.ipos[0] = state.getPosVal()[0];
        this.ipos[1] = state.getPosVal()[1];

        this.stime = (double) System.currentTimeMillis() / 1000;
        this.init_dist = getAdist(state);
    }

    public boolean distExit(MainState state) {
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        if (tdist < 0.2) {
            if (Math.abs(state.getHeadingVal() - thead) < 0.1) {
                if (SimpleMat.mag(state.getVelVal()) < 10) {
                    return true;
                }
            }
        }
        return false;
    }

    public boolean exit(MainState state) {
        if (this.pam) {
            return true;
        }
        return false;
    }

    public Command firePAM(MainState state) {
        // If angle is positive, must turn right, neg angvel
        // If angle is negative must turn left, pos angvel
        double x_theta = state.getLimeAVal()[0];
        double mag = Math.abs(x_theta) * 1 / (27);
        if (mag < .4) {
            mag = .4;
        }
        if (mag > 1) {
            mag = 1;
        }
        double ang_vel = mag;
        if (x_theta < 0) {
            ang_vel = ang_vel * -1;
        }
        double[] whl_array = MecanumIK.mecanumIK(new double[] { 0, 0 }, ang_vel);
        double flr = this.fl.updateRaw(whl_array[0], state.getFLRadssVal());
        double frr = this.fr.updateRaw(whl_array[1], state.getFRRadssVal());
        double blr = this.bl.updateRaw(whl_array[2], state.getBLRadssVal());
        double brr = this.br.updateRaw(whl_array[3], state.getBRRadssVal());

        double[] ocmd = { flr, frr, blr, brr, 0, 0, 0, 0, 0, 0, 0, 0 };
        return new Command(ocmd);
    }
            
            
    public Command update(MainState state) {
        this.tpos = SimpleMat.scaleVec(SimpleMat.unitVec(state.getPosVal()), Constants.FIRING_DIST);
        this.thead = SimpleMat.angleRectifier(SimpleMat.vecsAngle2(new double[] { 0, 1 }, tpos));

        if (distExit(state)) {
            this.pam = true;
        }
        Command main_cmd;
        if (pam) {
            main_cmd = firePAM(state);
        } else {
            main_cmd = trajectory(state, false, 0);
        }
        return main_cmd;
    }
}
