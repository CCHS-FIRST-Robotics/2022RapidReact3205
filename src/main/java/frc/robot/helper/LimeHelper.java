package frc.robot.helper;

import frc.robot.Constants;
import frc.robot.network.Network;
import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeHelper {
    public static double[] getPos(MainState state, Network net) {
        double deg2rad = 2 * Math.PI / 360;

        double[] ang_arr = net.lime.getAngles();
        //SmartDashboard.putNumberArray("Limelight/ang_arr", ang_arr);

        double elev_ang = (ang_arr[1] + Constants.LIME_ELEV_ANG) * deg2rad;
        double elev_height = Constants.LIME_RING_HEIGHT - Constants.LIME_LPOS[1];

        // tan elev * run = heihgt
        double horiz_dist = elev_height / Math.tan(elev_ang);
        SmartDashboard.putNumber("Limelight/horiz_dist", horiz_dist);
        // project heading
        double horiz_angle = SimpleMat.angleRectifier(state.getHeadingVal() + Math.PI - ang_arr[0] * deg2rad);
        //SmartDashboard.putNumber("Limelight/horiz_angle", horiz_angle);
        double[] dir_vec = SimpleMat.projectHeading(horiz_angle,
                horiz_dist + Constants.LIME_RING_RAD);
        // pos + dir_vec should be origin. Offset is pos offset
        double[] offset = SimpleMat.add(state.getPosVal(), dir_vec);
        offset = SimpleMat.add(offset, SimpleMat.projectHeading(state.getHeadingVal(), -1 * Constants.LIME_LPOS[0]));
        double[] new_pos = SimpleMat.subtract(state.getPosVal(), offset);
        //SmartDashboard.putNumberArray("Limelight/new_pos", new_pos);
        return new_pos;
    }
}
