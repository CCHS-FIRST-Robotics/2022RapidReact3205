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
        if (hardware.beam_1.get()) {
            state.setBeam1(0, 1);
        } else {
            state.setBeam1(1, 1);
        }
        if (hardware.beam_0.get()) {
            state.setBeam0(0, 1);
        } else {
            state.setBeam0(1, 1);
        }
        SmartDashboard.putBoolean("IRB/beam 0", hardware.beam_0.get());
        SmartDashboard.putBoolean("IRB/beam 1", hardware.beam_1.get());
    }
}
