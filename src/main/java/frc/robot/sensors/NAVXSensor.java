package frc.robot.sensors;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;
import frc.robot.helper.*;

public class NAVXSensor extends BaseSensor {
    double ang_var;
    public NAVXSensor(double sync_time){
        this.ang_var = Constants.BASE_HEADING_VAR;
        this.SYNC_TIME = sync_time;
    }
    public boolean shouldUse(){
        return true;
    }
    public void reset(HardwareObjects hardware){
        hardware.NAVX.reset();
        hardware.NAVX.calibrate();
        while (hardware.NAVX.isCalibrating()){

        }
        hardware.NAVX.zeroYaw();
        hardware.NAVX.setAngleAdjustment(0);
    }

    void updateHeadingVar() {
        this.ang_var = this.ang_var + Constants.DELTA_VAR * Constants.MAIN_DT;
        if (this.ang_var > Constants.MAX_HEADING_VAR) {
            this.ang_var = Constants.MAX_HEADING_VAR;
        }
    }

    public void processValue(MainState state, HardwareObjects hardware){
        updateHeadingVar();

        double heading = hardware.NAVX.getFusedHeading() * 2 * Math.PI/360;
        heading = SimpleMat.angleRectifier(heading);
        double[] global_acc = {hardware.NAVX.getWorldLinearAccelX(), hardware.NAVX.getWorldLinearAccelY()};
        global_acc = SimpleMat.scaleVec(global_acc, 9.81);
        
        double[] a2 = state.kalman2Update(state.getAccVal(), state.getAccVar(), global_acc, Constants.IMU_ACC_VAR);
        double[] h2 = state.kalmanUpdate(state.getHeadingVal(), state.getHeadingVar(), heading, ang_var);
        
        double[] new_acc = {a2[0], a2[1]};
        state.setAcc(new_acc, a2[2]);
        state.setHeading(h2[0],h2[1]);
        
        SmartDashboard.putNumber("Heading", heading);

        SmartDashboard.putNumber("Acc x", global_acc[0]);
        SmartDashboard.putNumber("Acc y", global_acc[1]);
    }
}