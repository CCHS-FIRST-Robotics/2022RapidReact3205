package frc.robot.ai.subroutines;

import java.util.ArrayList;

public class CurveFwdTravel extends Travel {
    ArrayList<double[]> point_list;

    public CurveFwdTravel(ArrayList<double[]> point_list) {
        this.point_list = point_list;
    }

}
