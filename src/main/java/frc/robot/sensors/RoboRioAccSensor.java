package frc.robot.sensors;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RoboRioAccSensor extends BaseSensor {
    public RoboRioAccSensor(double SYNC_TIME) {
        this.SYNC_TIME = SYNC_TIME;
    }

    public boolean shouldUse() {
        return true;
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double[] acc_vec = { hardware.RR_ACC.getX(), hardware.RR_ACC.getY() };
        acc_vec = SimpleMat.scaleVec(acc_vec, Constants.GRAV_ACC);

        SmartDashboard.putNumberArray("RR Acc/RR Acc", acc_vec);

        acc_vec = SimpleMat.rot2d(acc_vec, state.getHeadingVal());
        double var = Constants.IMU_ACC_VAR * 2;
        double[] a2 = state.kalman2Update(state.getAccVal(), state.getAccVar(), acc_vec, var);
        double[] new_acc = { a2[0], a2[1] };
        state.setAcc(new_acc, a2[2]);
    }
}