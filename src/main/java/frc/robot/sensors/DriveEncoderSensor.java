package frc.robot.sensors;

import frc.robot.sensors.BaseSensor;
import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.helper.SimpleMat;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

public class DriveEncoderSensor extends BaseSensor {
    public DriveEncoderSensor(double sync_time) {
        this.LAG_TIME = 0.0; // No Lag
        this.SYNC_TIME = sync_time;
        this.last_updated = sync_time;
    }

    public boolean shouldUse() {
        return true;
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double fl_raw = hardware.FLD_MOTOR.getSelectedSensorVelocity(1);
        double fr_raw = hardware.FRD_MOTOR.getSelectedSensorVelocity(1);
        double bl_raw = hardware.BLD_MOTOR.getSelectedSensorVelocity(1);
        double br_raw = hardware.BRD_MOTOR.getSelectedSensorVelocity(1);
        SmartDashboard.putNumber("fl raw", fl_raw);
        double dps2rads = 10 * 2 * Math.PI / 4096;
        fl_raw = fl_raw * dps2rads * -1;
        fr_raw = fr_raw * dps2rads * 1;
        bl_raw = bl_raw * dps2rads * -1;
        br_raw = br_raw * dps2rads * 1;
        state.setFLRadss(fl_raw, Constants.VAR_RAD_VAR);
        state.setFRRadss(fr_raw, Constants.VAR_RAD_VAR);
        state.setBLRadss(bl_raw, Constants.VAR_RAD_VAR);
        state.setBRRadss(br_raw, Constants.VAR_RAD_VAR);

        double[] pos_res = state.kalman2Update(state.getPosVal(), state.getPosVar(), state.getWhlOPosVal(),
                state.getWhlOPosVar());
        double[] vel_res = state.kalman2Update(state.getVelVal(), state.getVelVar(), state.getWhlOVelVal(),
                state.getWhlOVelVar());
        double[] h_res = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(),
                state.getWhlOHeadingVal(), state.getWhlOHeadingVar());
        double[] avel_res = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(), state.getWhlOAngVelVal(),
                state.getWhlOAngVelVar());
        double[] npos = { pos_res[0], pos_res[1] };
        double[] nvel = { vel_res[0], vel_res[1] };
        state.setPos(npos, pos_res[2]);
        state.setVel(nvel, vel_res[2]);
        state.setHeading(h_res[0], h_res[1]);
        state.setAngVel(avel_res[0], avel_res[1]);
    }
}
