package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.Constants;

public class Travel {

    PID fl;
    PID fr;
    PID bl;
    PID br;

    double[] tpos = { 0, 0 };
    double thead = 0;

    double[] ipos = { 0, 0 };

    public Travel(double[] target_pos, double target_heading) {
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

    public Command update(MainState state) {
        double theta = this.thead - state.getHeadingVal();
        double tdist = SimpleMat.mag(SimpleMat.subtract(state.getPosVal(), this.tpos));
        double adist;
        if (theta == 0) {
            adist = tdist;
        } else {
            adist = theta * tdist / (2 * Math.sin(theta / 2));
        }
        return new Command(0, 0, 0, 0);
    }
}
