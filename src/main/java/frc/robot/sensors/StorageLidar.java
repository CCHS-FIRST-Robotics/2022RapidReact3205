package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareObjects;
import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;

public class StorageLidar extends BaseSensor {
    public StorageLidar(double sync_time) {
        this.SYNC_TIME = sync_time;
    }

    public void reset(HardwareObjects hardware) {
        hardware.S_LIDAR.reset();
    }

    public boolean shouldUse() {
        return true;
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double dist;
        if (hardware.S_LIDAR.get() < 1) {
            dist = 0;
        } else {
            dist = (hardware.S_LIDAR.getPeriod() * 1000000 / 10) - 10;
        }
        state.setSLIDAR(dist, state.getSLIDARVar());
        //SmartDashboard.putNumber("S_LIDAR/dist", dist);
        //SmartDashboard.putNumber("S_LIDAR/getPeriod", hardware.S_LIDAR.getPeriod());
        //SmartDashboard.putNumber("S_LIDAR/get", hardware.S_LIDAR.get());
        //SmartDashboard.putNumber("S_LIDAR/getRate", hardware.S_LIDAR.getRate());
    }
}
