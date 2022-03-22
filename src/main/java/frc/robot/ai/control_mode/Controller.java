package frc.robot.ai.control_mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.ai.subroutines.*;
import frc.robot.helper.*;
import frc.robot.map.*;
import java.lang.Math;

import frc.robot.commands.Command;

public class Controller {
    XboxController xbox = new XboxController(Constants.XBOX_PORT);
    XboxController e_xbox = new XboxController(Constants.XBOX2_PORT);

    IntakeHandler intake;
    ShooterHandler shooter;

    PointAtMiddle pam;
    FiringPosition arty;
    BallChase chase;

    HangAlign halign;

    int pam_s = 0;
    int arty_s = 0;
    int chase_s = 0;
    int hang_s = 0;

    double pprop = 1;

    Curve sfr_curve;
    Curve sfl_curve;

    DPID fl_pid;
    DPID fr_pid;
    DPID bl_pid;
    DPID br_pid;

    double temp_cmd = 1;

    double start_time = System.currentTimeMillis() / 1000;

    double cooldown_time = System.currentTimeMillis() / 1000;

    public Controller() {
        this.sfr_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);
        this.sfl_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);

        resetPID();

        this.intake = new IntakeHandler();
        this.intake.idle();

        this.shooter = new ShooterHandler();
        this.shooter.idle();
    }

    void resetPID(){
        this.fl_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }

    double stickCurve(double val) {
        if (Math.abs(val) < 0.2) {
            val = 0;
        }
        return val;
    }

    double[] starControl(MainState state) {
        // left controllers give a polar vec, angle from 0 and mag from 0 to 1
        double lstick_x = xbox.getLeftX() * pprop;
        double lstick_y = 0;
        if (Math.abs(xbox.getLeftY()) > Math.abs(xbox.getRightY())) {
            lstick_y = xbox.getLeftY();
        } else {
            lstick_y = xbox.getRightY();
        }
        lstick_y = lstick_y * pprop;
        double rstick_x = xbox.getRightX() * pprop;
        double[] stick_vec = { lstick_x, -1 * lstick_y };
        double yaw_val = rstick_x;
        double raw_theta = SimpleMat.vecsAngle2(new double[] { 0, 1 }, stick_vec);
        double factor = 1;
        if (raw_theta > -1 * Math.PI / 4 && raw_theta < Math.PI / 4) {
            factor = Math.cos(raw_theta);
        } else if (raw_theta > Math.PI / 4 && raw_theta < 3 * Math.PI / 4) {
            factor = Math.cos(raw_theta - Math.PI / 2);
        } else if (raw_theta > 3 * Math.PI / 4 && raw_theta < Math.PI) {
            factor = Math.cos(raw_theta - Math.PI);
        } else if (raw_theta > -3 * Math.PI / 4 && raw_theta < -1 * Math.PI / 4) {
            factor = Math.cos(raw_theta + Math.PI / 2);
        } else if (raw_theta > -1 * Math.PI && raw_theta < -3 * Math.PI / 4) {
            factor = Math.cos(raw_theta + Math.PI);
        }
        double theta = raw_theta; // relative to the field
        double mag = SimpleMat.mag(stick_vec) * factor;

        double max_vel = Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        double max_avl = max_vel / (Constants.ROBOT_WIDTH / 2);
        double rthe = SimpleMat.angleRectifier(theta);
        max_vel = max_vel * Math.max(Math.abs(Math.cos(theta)), Math.abs(Math.sin(theta)));
        double[] vel_vec = SimpleMat.projectHeading(rthe, mag * max_vel);
        SmartDashboard.putNumberArray("Controller/Vel Vec", vel_vec);
        SmartDashboard.putNumber("Controller/mag", mag);

        double avel = max_avl * -1 * yaw_val;
        SmartDashboard.putNumber("Controller/AVEL", avel);
        double[] whl_vec = MecanumIK.mecanumIK(vel_vec, avel);
        SmartDashboard.putNumberArray("Controller/MIKC", whl_vec);
        return whl_vec;
    }

    public Command getCommands(MainState state, Map map, double CST) {
        double lr_strafe = 0;
        double fb_1 = 0;
        double lr_turn = 0;
        double fb_2 = 0;
        double intake = 0;
        double storage = 0;
        double storage_2 = 0;
        double shooter_1 = 0;
        double shooter_2 = 0;

        double hang_l = 0;
        double hang_r = 0;

        double[] pam_cmd = { 0, 0, 0, 0 };
        double[] arty_cmd = { 0, 0, 0, 0 };
        double[] chase_cmd = { 0, 0, 0, 0 };
        double[] hang_cmd = { 0, 0, 0, 0 };

        if (DriverStation.isTeleop()) {
            this.pprop = 1 - xbox.getLeftTriggerAxis() * 0.6 - e_xbox.getLeftTriggerAxis() * 0.2
                    - e_xbox.getRightTriggerAxis() * 0.2;

            lr_turn = 0;
            fb_1 = 0;

            intake = xbox.getRightTriggerAxis() * 0.6 + e_xbox.getRightX();
            ;
            storage = xbox.getRightTriggerAxis() * 0.6 - e_xbox.getRightY();
            ;

            storage_2 = e_xbox.getLeftY() * -1;

            shooter_1 = e_xbox.getLeftX();
            shooter_2 = e_xbox.getLeftX();

            hang_l = 0;
            hang_r = 0;
            if (e_xbox.getAButton()) {
                hang_l = -0.6;
                hang_r = -0.6;
            }
            if (e_xbox.getBButton()) {
                hang_l = 0.6;
                hang_r = 0.6;
            }

            if (e_xbox.getXButtonReleased()) {
                if (this.hang_s == 0) {
                    this.halign = new HangAlign(0.3);
                    this.hang_s = 1;
                } else {
                    this.hang_s = 0;
                }
            }

            if (xbox.getRightBumper()) {
                if (this.intake.substate == 0) {
                    this.intake.autoIntake(state);
                } else {
                    this.intake.idle();
                }
            }
            if (xbox.getAButton()) {
                storage_2 = storage_2 - 0.4;
            }
            if (xbox.getBButton()) {
                storage_2 = storage_2 + 0.4;
            }
            if (xbox.getYButtonReleased()) {
                if (this.shooter.state == 0) {
                    this.shooter.initDoubleFiring();
                } else {
                    this.shooter.idle();
                }
            }
            if (xbox.getXButtonReleased()) {
                if (this.shooter.state == 0) {
                    this.shooter.initFiring();
                } else {
                    this.shooter.idle();
                }
            }

            if (xbox.getLeftStickButtonReleased()) {
                if (this.arty_s == 0) {
                    this.arty = new FiringPosition(state, 0.3);
                    this.arty_s = 1;

                    this.pam_s = 0;
                    this.chase_s = 0;
                } else {
                    this.arty_s = 0;
                }
            }
            if (xbox.getRightStickButtonReleased()) {
                if (this.pam_s == 0) {
                    this.pam = new PointAtMiddle(state, 0.2);
                    this.pam_s = 1;

                    this.arty_s = 0;
                    this.chase_s = 0;
                } else {
                    this.pam_s = 0;
                }
            }
            if (xbox.getLeftBumperReleased()) {
                this.intake.idle();
                if (this.chase_s == 0) {
                    this.intake.autoIntake(state);
                    this.chase = new BallChase(state, map, -1, 0.4);
                    this.chase_s = 1;

                    this.arty_s = 0;
                    this.pam_s = 0;
                } else {
                    this.intake.idle();
                    this.chase_s = 0;
                }
            }
            if (this.pam_s == 1) {
                Command tp_cmd = this.pam.update(state);
                pam_cmd[0] = tp_cmd.fl_pprop;
                pam_cmd[1] = tp_cmd.fr_pprop;
                pam_cmd[2] = tp_cmd.bl_pprop;
                pam_cmd[3] = tp_cmd.br_pprop;
                if (this.pam.exit(state)) {
                    this.pam_s = 0;
                    resetPID();
                    this.temp_cmd = 0;
                    this.cooldown_time = System.currentTimeMillis() / 1000; 
                }
            }
            if (this.arty_s == 1) {
                Command ta_cmd = this.arty.update(state);
                arty_cmd[0] = ta_cmd.fl_pprop;
                arty_cmd[1] = ta_cmd.fr_pprop;
                arty_cmd[2] = ta_cmd.bl_pprop;
                arty_cmd[3] = ta_cmd.br_pprop;
                if (this.arty.exit(state)) {
                    this.arty_s = 0;
                    resetPID();
                    this.temp_cmd = 0;
                    this.cooldown_time = System.currentTimeMillis() / 1000; 
                }
            }
            if (this.chase_s == 1) {
                Command tc_cmd = this.chase.update(state, map);
                chase_cmd[0] = tc_cmd.fl_pprop;
                chase_cmd[1] = tc_cmd.fr_pprop;
                chase_cmd[2] = tc_cmd.bl_pprop;
                chase_cmd[3] = tc_cmd.br_pprop;
                boolean chase_exit = this.chase.exit(state, map);
                SmartDashboard.putBoolean("BallChase/chase exit", chase_exit);
                if (chase_exit) {
                    this.chase_s = 0;
                    resetPID();
                    this.temp_cmd = 0;
                    this.cooldown_time = System.currentTimeMillis() / 1000; 
                }
            }
            if (this.hang_s == 1) {
                Command th_cmd = this.halign.update(state);
                hang_cmd[0] = th_cmd.fl_pprop;
                hang_cmd[1] = th_cmd.fr_pprop;
                hang_cmd[2] = th_cmd.bl_pprop;
                hang_cmd[3] = th_cmd.br_pprop;
                boolean halign_exit = this.halign.exit(state);
                if (halign_exit) {
                    this.hang_s = 0;
                    resetPID();
                    this.temp_cmd = 0;
                    this.cooldown_time = System.currentTimeMillis() / 1000; 
                }
            }

            double[] ins_cmd = this.intake.update(state);
            double[] sho_cmd = this.shooter.update(state);
            if (this.intake.substate != 0) {
                intake = ins_cmd[0];
                storage = ins_cmd[1];
                storage_2 = ins_cmd[2] + sho_cmd[0];
            }
            if (this.shooter.state != 0) {
                intake = ins_cmd[0] + sho_cmd[3];
                storage = ins_cmd[1] + sho_cmd[3];
                storage_2 = ins_cmd[2] + sho_cmd[0];
                shooter_1 = sho_cmd[1];
                shooter_2 = sho_cmd[2];
            }

        }
        double[] whl_vec = starControl(state);
        if (System.currentTimeMillis()/1000 - cooldown_time > 1){
            this.temp_cmd = 1;
        }
        else if (System.currentTimeMillis()/1000 - cooldown_time < 0.9){
            SmartDashboard.putNumber("Controller/end I", this.fl_pid.integral);
        }
        else{
            this.temp_cmd = 0;
            resetPID();
            SmartDashboard.putNumber("Controller/end I2", this.fl_pid.integral);
            SmartDashboard.putNumberArray("Controller/end starcmd", whl_vec);
        }
        // double flt = whl_vec[0];
        // double frt = whl_vec[1];
        // double blt = whl_vec[2];
        // double brt = whl_vec[3];
        double flt = whl_vec[0];// + (Math.min(1, Math.max(-1, flt))) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        double frt = whl_vec[1];// + (Math.min(1, Math.max(-1, frt))) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        double blt = whl_vec[2];// + (Math.min(1, Math.max(-1, blt))) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        double brt = whl_vec[3];// + (Math.min(1, Math.max(-1, brt))) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;

        double fld = flt - state.getFLRadssVal();
        double frd = frt - state.getFRRadssVal();
        double bld = blt - state.getBLRadssVal();
        double brd = brt - state.getBRRadssVal();

        double flr = 0;
        double frr = 0;
        double blr = 0;
        double brr = 0;
        if (chase_s == 0 && pam_s == 0 && arty_s == 0 && hang_s == 0){
            flr = this.fl_pid.update(fld)*temp_cmd;
            frr = this.fr_pid.update(frd)*temp_cmd;
            blr = this.bl_pid.update(bld)*temp_cmd;
            brr = this.br_pid.update(brd)*temp_cmd;
        }
        else {
        flr = pam_cmd[0] + arty_cmd[0] + chase_cmd[0] + hang_cmd[0];
        frr = pam_cmd[1] + arty_cmd[1] + chase_cmd[1] + hang_cmd[1];
        blr = pam_cmd[2] + arty_cmd[2] + chase_cmd[2] + hang_cmd[2];
        brr = pam_cmd[3] + arty_cmd[3] + chase_cmd[3] + hang_cmd[3];
        }

        if (System.currentTimeMillis()/1000 - cooldown_time < 1){
            SmartDashboard.putNumber("Controller/end flr", this.fl_pid.update(fld));
            SmartDashboard.putNumber("Controller/end fld", fld);
        }
        //flr = flr*temp_cmd;
        //frr = frr*temp_cmd;
        //blr = blr*temp_cmd;
        //brr = brr*temp_cmd;

        e_xbox.setRumble(RumbleType.kLeftRumble, 0.1);

        // rumble based on voltage , 1 at 7 and 0 at 11
        double voltage = RobotController.getBatteryVoltage();
        double rmb = 1 - ((voltage - 7) / 4);
        rmb = Math.min(Math.max(0, rmb), 1);
        xbox.setRumble(RumbleType.kRightRumble, rmb);

        double[] ocmd = { flr, frr, blr, brr, intake, storage, storage_2, shooter_1, shooter_2, hang_l, hang_r };
        SmartDashboard.putNumberArray("Controller/cmd", ocmd);
        Command command = new Command(ocmd);
        // Command command = new Command(flt*0.1, frt*0.1, blt*0.1,brt*0.1);
        return command;
    }
}