package frc.robot.sensors;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;
import frc.robot.helper.*;

public class NAVXAccumSensor extends BaseSensor {
    double[] pos = { 0, 0 };
    double heading = 0;
    double prev_time = 0;

    public NAVXAccumSensor(MainState state, HardwareObjects hardware, double sync_time) {
        this.SYNC_TIME = sync_time;
        reset(state, hardware);
    }

    public boolean shouldUse(HardwareObjects hardware) {
        if(hardware.NAVX.isConnected() == false)
            return false;
        if(hardware.NAVX.isCalibrating() == true)
            return false;

        return true;
    }

    public void reset(MainState state, HardwareObjects hardware) {
        this.pos[0] = state.getPosVal()[0];
        this.pos[1] = state.getPosVal()[1];
        this.heading = state.getHeadingVal();
        this.prev_time = (double) System.currentTimeMillis() / 1000;
        hardware.NAVX.resetDisplacement();
    }

    boolean shouldReset(MainState state) {
        // not moving
        double vel_mag = SimpleMat.mag(state.getVelVal()) + SimpleMat.mag(state.getAccVal()) * Constants.MAIN_DT;
        double avel_mag = Math.abs(state.getAngVelVal()) + Math.abs(state.getAngAccVal()) * Constants.MAIN_DT;
        if (vel_mag < 0.2 && avel_mag < 0.2) {
            return true;
        }
        return false;
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        if (shouldReset(state)) {
            reset(state, hardware);
        }

        double dt = (double) System.currentTimeMillis() / 1000;
        dt = dt - prev_time;

        double pos_var = Constants.IMU_ACC_VAR * Math.pow(dt + 1, 2);
        double vel_var = Constants.IMU_ACC_VAR * (dt + 1);

        double[] ld = { hardware.NAVX.getDisplacementX(), hardware.NAVX.getDisplacementY() * -1 };
        double[] lv = { hardware.NAVX.getVelocityX(), hardware.NAVX.getVelocityY() * -1 };

        double[] est_pos = SimpleMat.add(this.pos, SimpleMat.rot2d(ld, this.heading));
        double[] est_vel = SimpleMat.rot2d(lv, this.heading);

        double[] kpi = state.kalman2Update(state.getPosVal(), state.getPosVar(), est_pos, pos_var);
        double[] kvi = state.kalman2Update(state.getVelVal(), state.getVelVar(), est_vel, vel_var);

        double[] new_pos = { kpi[0], kpi[1] };
        double[] new_vel = { kvi[0], kvi[1] };

        SmartDashboard.putNumberArray("Accum/ld", ld);
        SmartDashboard.putNumberArray("Accum/lv", lv);

        SmartDashboard.putNumberArray("Accum/pos", est_pos);
        SmartDashboard.putNumberArray("Accum/vel", est_vel);

        state.setPos(new_pos, kpi[2]);
        state.setVel(new_vel, kvi[2]);

    }
}
