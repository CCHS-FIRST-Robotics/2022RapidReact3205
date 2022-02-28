package frc.robot.ai.subroutines;

import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.helper.PID;

public class ShooterHandler {

    PID storage_2;
    PID shooter_1;
    PID shooter_2;

    public int state = 0;

    double o_time = System.currentTimeMillis() / 1000;

    public ShooterHandler() {
        this.state = 0; // 0: All PID to 0 ; 1: Spooling, CIM PID to t ; 2: Firing, all to t
        this.storage_2 = new PID(Constants.R_STRONG_PID[0], Constants.R_STRONG_PID[1], Constants.R_STRONG_PID[2]);
        this.shooter_1 = new PID(Constants.C_STRONG_PID[0], Constants.C_STRONG_PID[1], Constants.C_STRONG_PID[2]);
        this.shooter_2 = new PID(Constants.C_STRONG_PID[0], Constants.C_STRONG_PID[1], Constants.C_STRONG_PID[2]);
    }

    public void initFiring() {
        this.state = 1;
        this.o_time = System.currentTimeMillis() / 1000;
    }

    public void idle() {
        this.state = 0;
    }

    public double[] update(MainState state) {
        double rpm2radss = 2 * Math.PI / 60;

        double so2_target = 0;
        double sh1_target = 0;
        double sh2_target = 0;

        if (this.state != 0) {
            sh1_target = Constants.SHOOTER_1_RPM * rpm2radss;
            sh2_target = Constants.SHOOTER_2_RPM * rpm2radss;
        }
        if (this.state == 1) {
            double dt = (System.currentTimeMillis() / 1000) - this.o_time;
            if (dt > 0.5) {
                this.state = 2;
            }
        }
        if (this.state == 2) {
            so2_target = Constants.STORAGE_2_RPM * rpm2radss;
        }

        double so2_resp = this.storage_2.update(so2_target - state.getStorage2Val());
        double sh1_resp = this.shooter_1.update(sh1_target - state.getShooterVal()[0]);
        double sh2_resp = this.shooter_2.update(sh2_target - state.getShooterVal()[1]);

        if (this.state == 0) {
            double[] output = { 0, 0, 0 };
            return output;
        }
        double[] output = { so2_resp, sh1_resp, sh2_resp };
        return output;
    }

}
