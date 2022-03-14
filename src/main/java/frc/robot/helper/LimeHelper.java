package frc.robot.helper;

import frc.robot.Constants;
import frc.robot.network.Network;
import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    public static double[] getPos(MainState state, Network net) {
        double deg2rad = 2 * Math.PI / 360;

        double[] ang_arr = net.lime.getAngles();

        double elev_ang = (ang_arr[1] + Constants.LIME_ELEV_ANG) * deg2rad;
        double elev_height = Constants.LIME_RING_HEIGHT - Constants.LIME_LPOS[1];

        // tan elev * run = heihgt
        double horiz_dist = elev_height / Math.tan(elev_ang);
        SmartDashboard.putNumber("Limelight/horiz_dist", horiz_dist);
        // project heading
        double horiz_angle = SimpleMat.angleRectifier(state.getHeadingVal() + Math.PI - ang_arr[0] * deg2rad);
        SmartDashboard.putNumber("Limelight/horiz_angle", horiz_angle);
        double[] dir_vec = SimpleMat.projectHeading(horiz_angle,
                horiz_dist + Constants.LIME_LPOS[0] + Constants.LIME_RING_RAD);
        // pos + dir_vec should be origin. Offset is pos offset
        double[] offset = SimpleMat.add(state.getPosVal(), dir_vec);
        double[] new_pos = SimpleMat.subtract(state.getPosVal(), offset);
        return new_pos;
    }
}
