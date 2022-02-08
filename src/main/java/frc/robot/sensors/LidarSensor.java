package frc.robot.sensors;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.network.Network;

public class LidarSensor extends BaseSensor {

    public LidarSensor(double sync_time) {
        this.SYNC_TIME = sync_time;
    }

    public boolean shouldUse() {
        return true;
    }

    public void reset() {

    }

    public void processValue(MainState state, Network network) {
        double[] pos = network.lidar.getPosVal();
        double heading = network.lidar.getHeadingVal();
        double[] npa = state.kalman2Update(state.getPosVal(), state.getPosVar(), pos, Constants.LIDAR_P_VAR);
        double[] ha = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(), heading,
                Constants.LIDAR_H_VAR);
        double[] new_pos = { npa[0], npa[1] };
        state.setPos(new_pos, pos[2]);
        state.setHeading(ha[0], ha[1]);
    }
}
