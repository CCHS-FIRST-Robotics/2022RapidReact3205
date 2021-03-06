package frc.robot.ai.subroutines;

import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.helper.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterHandler {

    DPID storage_2;
    DPID shooter_1; // front motor
    DPID shooter_2; // back motor

    public int state = 0;

    double o_time = System.currentTimeMillis() / 1000;
    double d_time = System.currentTimeMillis() / 1000;

    public ShooterHandler() {
        this.state = 0; // 0: All PID to 0 ; 1: Spooling, CIM PID to t ; 2: Firing, all to t
        // For firing both 3 spooling both, 4 firing 1, 5 firing both
        this.storage_2 = new DPID(Constants.R_STRONG_PID[0], Constants.R_STRONG_PID[1], Constants.R_STRONG_PID[2]);
        this.shooter_1 = new DPID(Constants.C_STRONG_PID[0], Constants.C_STRONG_PID[1], Constants.C_STRONG_PID[2]);
        this.shooter_2 = new DPID(Constants.C_STRONG_PID[0], Constants.C_STRONG_PID[1], Constants.C_STRONG_PID[2]);
    }

    public void initFiring() {
        this.state = 1;
        this.o_time = System.currentTimeMillis() / 1000;
        this.shooter_1.softReset();
        this.shooter_2.softReset();
    }

    public void initDoubleFiring() {
        this.state = 3;
        this.o_time = System.currentTimeMillis() / 1000;
        this.shooter_1.softReset();
        this.shooter_2.softReset();
    }

    public void initWallFiring() {
        this.state = 6;
        this.o_time = System.currentTimeMillis() / 1000;
        this.shooter_1.softReset();
        this.shooter_2.softReset();
    }

    public void initAuton2Firing() {
        this.state = 8;
        this.o_time = System.currentTimeMillis() / 1000;
        this.shooter_1.softReset();
        this.shooter_2.softReset();
    }

    public void idle() {
        this.state = 0;
    }

    public double[] update(MainState state) {
        double ct = (System.currentTimeMillis() / 1000);
        double rpm2radss = 2 * Math.PI / 60;

        double so2_target = 0;  // storage target rpm
        double sh1_target = 0;  // shooter 1 target rpm
        double sh2_target = 0;  // shooter 2 target rpm

        double so2_resp = 0;
        double intake_resp = 0;

        if (this.state != 0) {
            sh1_target = 1 * Constants.SHOOTER_1_RPM * rpm2radss;
            sh2_target = 1 * Constants.SHOOTER_2_RPM * rpm2radss;
        }
        if (this.state == 1) {
            double dt = (System.currentTimeMillis() / 1000) - this.o_time;
            sh1_target = 0.8995 * Constants.SHOOTER_1_RPM * rpm2radss;
            sh2_target = 0.8995 * Constants.SHOOTER_2_RPM * rpm2radss;
            if (dt > 1.1) {
                this.state = 2;
                storage_2.softReset();
            }
            so2_target = 0;
        }
        if (this.state == 2) {
            so2_target = Constants.STORAGE_2_RPM * rpm2radss  - state.getStorage2Val();
            if (exit()) {
                this.state = 0;
                this.storage_2.softReset();
            }
        }
        if (this.state == 3) {
            double dt = ct - this.o_time;
            if (dt > 1) {
                this.state = 4;
                d_time = (System.currentTimeMillis() / 1000);
            }
            so2_target = 0.;
        }
        if (this.state == 4) {
            so2_target = Constants.STORAGE_2_RPM * rpm2radss  - state.getStorage2Val();
            if (ct - d_time > 1.5) {
                this.state = 5;
            }
        }
        
        if (this.state == 5) {
            sh1_target *= 0.975;
            sh2_target *= 0.975;
            so2_target = 1 * Constants.STORAGE_2_RPM * rpm2radss  - state.getStorage2Val();
            if (exit()) {
                this.state = 0;

            }
            intake_resp = 0.7;
        }

        if(this.state == 6) {
            double dt = ct - this.o_time;
            if(dt > 1.1) {
                this.state = 7;
                d_time = (System.currentTimeMillis() / 1000);
            }
            so2_target = 0.0;
        }

        if(this.state == 7) {
            sh1_target *= 0.6464; //0.791 -> 0.8
            sh2_target *= 0.791; //0.791 -> 0.8
            so2_target = Constants.STORAGE_2_RPM * rpm2radss  - state.getStorage2Val();
            if (ct - d_time > 1.6) {
                this.state = 9;
            }
           
        }

        if(this.state == 8) {
            sh1_target = sh1_target * 1.35;
            sh2_target = sh2_target * 1.35;
            double dt = (System.currentTimeMillis() / 1000) - this.o_time;
            if (dt > 1.1) {
                this.state = 2;
                storage_2.softReset();
            }
            so2_target = 0;
        }

        if (this.state == 9) {
            sh1_target *= 0.789;
            sh2_target *= 0.965;
            so2_target = 1 * Constants.STORAGE_2_RPM * rpm2radss  - state.getStorage2Val();
            if (exit()) {
                this.state = 0;

            }
            intake_resp = 0.7;
        }

        if (so2_target != 0){
            so2_resp = this.storage_2.update(so2_target);
        }
        double sh1_resp = Math.max(-1, Math.min(1, this.shooter_1.update(sh1_target - state.getShooterVal()[0])));
        double sh2_resp = Math.max(-1, Math.min(1, this.shooter_2.update(sh2_target - state.getShooterVal()[1])));
        //SmartDashboard.putNumber("Shooter/state", this.state);
        if (this.state == 0) {
            double[] output = { 0, 0, 0, 0 };
            return output;
        }
        double[] output = { so2_resp, sh1_resp, sh2_resp, intake_resp };
        return output;

    }

    public boolean exit() {
        if (state == 0) {
            return true;
        }
        double ct = (System.currentTimeMillis() / 1000);
        if (this.state == 2) {
            double dt = ct - this.o_time;
            if (dt > 3.5) {
                return true;
            }
        }
        if (this.state == 5 || this.state == 9) {
            if (ct - d_time > 4) {
                return true;
            }
        }
        return false;
    }

}
