package frc.robot.helper;

import frc.robot.Constants;
import frc.robot.network.Network;
import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeHelper {
    public static double getHorizDist(MainState state, Network net) {
        double deg2rad = 2 * Math.PI / 360;

        double[] ang_arr = net.lime.getAngles();
        // SmartDashboard.putNumberArray("Limelight/ang_arr", ang_arr);

        double elev_ang = (ang_arr[1] + Constants.LIME_ELEV_ANG) * deg2rad;
        double elev_height = Constants.LIME_RING_HEIGHT - Constants.LIME_LPOS[1];

        // tan elev * run = heihgt
        double horiz_dist = elev_height / Math.tan(elev_ang);
        return horiz_dist;
    }

    public static double[] getPos(MainState state, Network net) {
        
        double deg2rad = 2 * Math.PI / 360;

        double[] ang_arr = net.lime.getAngles();
        // tan elev * run = heihgt
        double horiz_dist = getHorizDist(state, net);
        SmartDashboard.putNumber("Limelight/horiz_dist", horiz_dist);
        // project heading
        double horiz_angle = SimpleMat.angleRectifier(state.getHeadingVal() + Math.PI - ang_arr[0] * deg2rad);
        // SmartDashboard.putNumber("Limelight/horiz_angle", horiz_angle);
        double[] dir_vec = SimpleMat.projectHeading(horiz_angle,
                horiz_dist + Constants.LIME_RING_RAD);
        // pos + dir_vec should be origin. Offset is pos offset
        double[] offset = SimpleMat.add(state.getPosVal(), dir_vec);
        offset = SimpleMat.add(offset, SimpleMat.projectHeading(state.getHeadingVal(), -1 * Constants.LIME_LPOS[0]));
        double[] new_pos = SimpleMat.subtract(state.getPosVal(), offset);
        // SmartDashboard.putNumberArray("Limelight/new_pos", new_pos);
        return new_pos;
    }

    public static boolean getOutTRange(MainState state, Network net) {
        
        double[] ang_arr = net.lime.getAngles();
        double deg2rad = 2 * Math.PI / 360;
        double horiz_dist = getHorizDist(state, net);
        double r_dist = 0.61;
        double angle = 1 * Math.atan(r_dist / (horiz_dist + r_dist));
        SmartDashboard.putNumber("Limelight/exit angle", 27 * deg2rad - angle);
        SmartDashboard.putNumber("Limelight/exit current", ang_arr[0]);
        if (Math.abs(ang_arr[0]) * deg2rad > 27 * deg2rad - angle) {
            return true;
        }
        return false;
    }

    public static boolean getBoxWrongDims(Network net) {
        double[] rect_arr = net.lime.getRect();
        // if vert height is more than 2/3 of horiz dist, then return true
        if (rect_arr[1] / rect_arr[0] > 2 / 3) {
            return true;
        }
        return false;
    }

}
