package frc.robot.ai.subroutines;

import frc.robot.helper.*;
import frc.robot.state.MainState;
import frc.robot.commands.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToPoint {

    PID fl;
    PID fr;
    PID bl;
    PID br;

    double[] tpos = { 0, 0 };
    double max_prop = 1;
    double angvel_max;
    double angacc_max;
    FwdController velcontr;

    public TurnToPoint(double[] tvec, double max_prop) {
        this.tpos[0] = tvec[0];
        this.tpos[1] = tvec[1];
        this.max_prop = max_prop;

        this.angvel_max = max_prop * 10 * Constants.ROBOT_WIDTH * Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * Math.PI
                / 60;

        double cross_tc = Math.sin(Math.PI * 0.75 - Math.atan(Constants.ROBOT_WIDTH / Constants.ROBOT_LENGTH));
        double[] p_vec = { Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH };
        double adist = SimpleMat.mag(p_vec);
        this.angacc_max = max_prop * adist * cross_tc * 4 * Constants.MOTOR_MAX_TORQUE
                / (Constants.WHEEL_RADIUS * Constants.MOI);

        SmartDashboard.putNumber("turn/angmax",this.angvel_max);
        this.velcontr = new FwdController(this.angvel_max, this.angacc_max);

        this.fl = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1]*1, 0.000);
        this.fr = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1]*1, 0.000);
        this.bl = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1]*1, 0.000);
        this.br = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1]*1, 0.000);
    }

    double getTheta(MainState main_state) {
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.tpos[0] - pos[0], this.tpos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double pwr_cmd = SimpleMat.vecsAngle2(unit_h_vec, point_vec);

        return pwr_cmd;
    }

    public boolean exit(MainState main_state) {
        double remaining = Math.abs(getTheta(main_state));
        if (remaining < Constants.ACCEPTABLE_ANGLE_ERROR) {
            return true;
        }
        return false;
    }

    public Command update(MainState state) {
        double dtheta = getTheta(state);
        double target_avel = this.velcontr.update(state.getAngVelVal(), dtheta);
        
        SmartDashboard.putNumber("turn/dtheta",dtheta);
        
        double[] vel = { 0, 0 };
        double[] whl_array = MecanumIK.mecanumIK(vel, target_avel);

        SmartDashboard.putNumber("turn/tavel",target_avel);
        SmartDashboard.putNumberArray("turn/whl arr",whl_array);

        double flr = this.fl.update(whl_array[0] - state.getFLRadssVal());
        double frr = this.fr.update(whl_array[1] - state.getFRRadssVal());
        double blr = this.bl.update(whl_array[2] - state.getBLRadssVal());
        double brr = this.br.update(whl_array[3] - state.getBRRadssVal());

        SmartDashboard.putNumber("turn/flr",flr);

        return new Command(flr, frr, blr, brr);
    }

}
