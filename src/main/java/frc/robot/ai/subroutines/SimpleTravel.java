package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleTravel extends Travel {
    public SimpleTravel(double[] target_pos, double target_heading, double max_prop) {

        this.v_max = max_prop * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
                / 60;
        this.a_max = max_prop * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.ROBOT_MASS);
        SmartDashboard.putNumber("vmax", this.v_max);
        SmartDashboard.putNumber("amax", this.a_max);
        this.v_contr = new FwdController(this.v_max, this.a_max);

        this.angvel_max = max_prop * Constants.ROBOT_WIDTH * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * Math.PI
                / 60;

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

    public boolean exit(MainState state) {
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        if (tdist < 0.1) {
            return true;
        }
        double[] tdiff = SimpleMat.subtract(state.getPosVal(), this.tpos);
        tdiff = SimpleMat.unitVec(tdiff);
        double[] sdiff = SimpleMat.subtract(this.ipos, this.tpos);
        sdiff = SimpleMat.unitVec(sdiff);
        if (SimpleMat.dot(tdiff, sdiff) < Math.cos(Math.PI * 0.25)) {
            if (tdist < 0.5) {
                return true;
            }
        }
        return false;
    }
}
