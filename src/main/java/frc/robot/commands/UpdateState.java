package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.helper.SimpleMat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UpdateState {

    public static double motorT(double whl_radss, double pwr_prop) {
        double rpm_prop = Math.abs(whl_radss) / (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60);
        double torque = pwr_prop * (1 - rpm_prop) * Constants.MOTOR_MAX_TORQUE;
        return torque;
    }

    public static void updateState(MainState state, Command command) {
        double flf = motorT(state.getFLRadssVal(), command.fl_pprop);
        double frf = motorT(state.getFRRadssVal(), command.fr_pprop);
        double blf = motorT(state.getBLRadssVal(), command.bl_pprop);
        double brf = motorT(state.getBRRadssVal(), command.br_pprop);

        state.setFLT(flf, Constants.INIT_VARIANCE);
        state.setFRT(frf, Constants.INIT_VARIANCE);
        state.setBLT(blf, Constants.INIT_VARIANCE);
        state.setBRT(brf, Constants.INIT_VARIANCE);

        flf = flf / Constants.WHEEL_RADIUS;
        frf = frf / Constants.WHEEL_RADIUS;
        blf = blf / Constants.WHEEL_RADIUS;
        brf = brf / Constants.WHEEL_RADIUS;

        double r2o2 = Math.sqrt(2) / 2;
        double fwd_force = (flf + frf + blf + brf) * r2o2;
        double side_force = (flf - frf - blf + brf) * r2o2;
        // assume com = center
        double cross_tc = Math.sin(Math.PI * 0.75 - Math.atan(Constants.ROBOT_WIDTH / Constants.ROBOT_LENGTH));
        double[] p_vec = { Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH };
        double adist = SimpleMat.mag(p_vec);
        double torque = cross_tc * adist * (-1 * flf + frf - blf + brf);
        double ang_acc = torque / Constants.MOI;
        // local to global;
        double[] acc_vec = { side_force / Constants.ROBOT_MASS, fwd_force / Constants.ROBOT_MASS };
        acc_vec = SimpleMat.rot2d(acc_vec, state.getHeadingVal());
        // var
        double ave_prop_coeff = 0.25 * (Math.abs(command.fl_pprop) + Math.abs(command.fr_pprop)
                + Math.abs(command.bl_pprop) + Math.abs(command.br_pprop));
        
        double acc_error = SimpleMat.mag(SimpleMat.subtract(acc_vec, state.getAccVal()));
        SmartDashboard.putNumber("acc_diff", acc_error);

        double[] nacc = state.kalman2Update(state.getAccVal(), state.getAccVar(), acc_vec, ave_prop_coeff * Constants.ACC_VARIANCE );
        double[] nacc_0 = {nacc[0], nacc[1]};
        //state.setAcc(nacc_0, nacc[2]);
        //state.setAngAcc(ang_acc, ave_prop_coeff * Constants.ANG_VEL_VARIANCE);
    }
}
