package frc.robot.sensors;

import frc.robot.Constants;
import frc.robot.helper.*;
import frc.robot.network.*;
import frc.robot.state.*;

public class Limelight extends BaseSensor {
    public Limelight(double sync_time) {
        this.SYNC_TIME = sync_time;
    }

    public boolean shouldUse(MainState state, Network net) {
        if (LimeHelper.getOutTRange(state, net)) {
            return false;
        }
        if (LimeHelper.getBoxWrongDims(net)) {
            return false;
        }
        if (net.lime.getRect()[0] < 10 || net.lime.getRect()[0] < 10) {
            return false;
        }
        if (net.lime.getValid()) {
            return true;
        }
        return false;
    }

    public void processValue(MainState state, Network net) {
        double[] new_pos = LimeHelper.getPos(state, net);
        double[] update = state.kalman2Update(state.getPosVal(), state.getPosVar(), new_pos, Constants.LIME_POS_VAR);
        double[] update_pos = { update[0], update[1] };
        state.setPos(update_pos, update[2]);
    }
}
