package frc.robot.helper;

import frc.robot.Constants;

public class MecanumIK {
    public static double[] mecanumIK(double[] vel, double avel){
        double ta = Math.tan(Math.PI/4);
        double tw = (Constants.ROBOT_WIDTH * ta + Constants.ROBOT_LENGTH)/(2 * ta);
        double fl = vel[1] + (1/ta) * vel[0] - tw * avel;
        double fr = vel[1] - (1/ta) * vel[0] + tw * avel;
        double bl = vel[1] - (1/ta) * vel[0] - tw * avel;
        double br = vel[1] + (1/ta) * vel[0] + tw * avel;

        fl = fl/Constants.WHEEL_RADIUS;
        fr = fr/Constants.WHEEL_RADIUS;
        bl = bl/Constants.WHEEL_RADIUS;
        br = br/Constants.WHEEL_RADIUS;

        double[] whl_radss = {fl, fr, bl, br};
        return whl_radss;
    }
}
