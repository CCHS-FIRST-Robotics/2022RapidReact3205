package frc.robot.helper;

import frc.robot.Constants;
import frc.robot.map.*;
import frc.robot.state.MainState;
import frc.robot.helper.*;

public class GetBall {
    public static int getClosestBall(MainState state, Map map) {
        double min_dist = 30;
        int min_ind = -1;
        for (int c = 0; c < Constants.BALL_NUM; c++) {
            Ball ball = map.balls[c];
            if (ball.state != 0) {
                double[] diff = SimpleMat.subtract(ball.pos, state.getPosVal());
                double dist = SimpleMat.mag(diff);
                double ang = Math
                        .acos(SimpleMat.dot(SimpleMat.unitVec(diff),
                                SimpleMat.projectHeading(state.getHeadingVal(), 1)));
                if (ang < 1) {
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_ind = c;
                    }
                }
            }
        }
        return min_ind;
    }
}