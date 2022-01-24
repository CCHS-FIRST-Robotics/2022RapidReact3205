package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.helper.SimpleMat;

public class UpdateState {
    public static double wheelForceFactor(double prop, double rel_vel) {
        double maxms = (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS;
        double factor = 1 - (rel_vel / maxms);
        if (factor > 1) {
            factor = 1;
        } else if (factor < 0) {
            factor = 0;
        }
        return factor;
    }

    public static double motorForce(MainState state, double pwr_prop) {
        double[] h_vec = SimpleMat.projectHeading(state.getHeadingVal(), 1);
        double rel_vel = SimpleMat.scalarProject(h_vec, state.getVelVal());

        double f = pwr_prop * Constants.MOTOR_MAX_TORQUE * wheelForceFactor(pwr_prop, rel_vel) / Constants.WHEEL_RADIUS;
        return f;
    }

    public static void updateState(MainState state, Command command){
        double flf = motorForce(state, command.fl_pprop);
        double frf = motorForce(state, command.fr_pprop);
        double blf = motorForce(state, command.bl_pprop);
        double brf = motorForce(state, command.br_pprop);

        double r2o2 = Math.sqrt(2)/2;
        double fwd_force = (flf + frf + blf + brf) * r2o2;
        double side_force = (flf - frf - blf + brf) * r2o2;
        //assume com = center
        double cross_tc = Math.asin(Math.PI * 0.75 - Math.atan(Constants.ROBOT_WIDTH/Constants.ROBOT_LENGTH));
        double[] p_vec = {Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH};
        double adist = SimpleMat.mag(p_vec);
        double torque = cross_tc * adist * (-1 * flf + frf - blf + brf);
        double ang_acc = torque / Constants.MOI;
        //local to global;
        double[] acc_vec = {side_force/Constants.ROBOT_MASS, fwd_force/Constants.ROBOT_MASS};
        acc_vec = SimpleMat.rot2d(acc_vec, state.getHeadingVal());
        //var
        double ave_prop_coeff = 0.25 * (Math.abs(command.fl_pprop) + Math.abs(command.fr_pprop) + Math.abs(command.bl_pprop) + Math.abs(command.br_pprop));
        state.setAcc(acc_vec, ave_prop_coeff * Constants.ACC_VARIANCE);
        state.setAngAcc(ang_acc, ave_prop_coeff * Constants.ANG_VEL_VARIANCE);
    }
}
