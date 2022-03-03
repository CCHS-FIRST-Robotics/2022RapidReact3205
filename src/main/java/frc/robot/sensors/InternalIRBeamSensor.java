package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareObjects;
import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;

public class InternalIRBeamSensor extends BaseSensor {
    public InternalIRBeamSensor(double sync_time) {
        this.SYNC_TIME = sync_time;
    }

    public void reset(HardwareObjects hardware) {
        hardware.S_LIDAR.reset();
    }

    public boolean shouldUse() {
        return true;
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        SmartDashboard.putBoolean("IRB/dist", hardware.beam_1.get());
    }
}
