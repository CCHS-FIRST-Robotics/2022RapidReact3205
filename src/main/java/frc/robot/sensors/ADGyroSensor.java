package frc.robot.sensors;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ADGyroSensor extends BaseSensor {

    public ADGyroSensor(double SYNC_TIME) {
        this.SYNC_TIME = SYNC_TIME;
    }

    public boolean shouldUse(HardwareObjects hardware) {
        if(hardware.AD_GYRO.isConnected() == false)
            return false;
        return true;
    }

    public void reset(HardwareObjects hardware) {
        hardware.AD_GYRO.reset();
        hardware.AD_GYRO.calibrate();
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double raw_gyro = (hardware.AD_GYRO.getRate() * -1 * 2 * Math.PI) / (0.0128 * 360);

        //.putNumber("AD Gyro/AD Gyro", raw_gyro);

        double var = Constants.MAX_HEADING_VAR / Constants.MAIN_DT;
        double[] av1 = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(), raw_gyro, var);
        // state.setAngVel(av1[0], av1[1]);
    }
}
