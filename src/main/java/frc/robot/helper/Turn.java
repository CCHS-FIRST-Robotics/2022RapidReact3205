package frc.robot.helper;

import frc.robot.state.MainState;

public class Turn {
    public PID turn = new PID(1.5, 7, 0.0);
    public double[] tpos = { 0, 0 };
    public double max_prop = 1;
    public double angvel_max;
    public double angacc_max;
    public double init_dtheta;
    public FwdController velcontr;
    public double stime;

    public double getTheta(MainState main_state) {
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.tpos[0] - pos[0], this.tpos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double pwr_cmd = SimpleMat.vecsAngle2(unit_h_vec, point_vec);

        return pwr_cmd;
    }
}
