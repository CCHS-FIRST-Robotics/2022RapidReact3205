package frc.robot.ai.control_mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.ai.subroutines.*;
import frc.robot.helper.*;
import java.lang.Math;

import frc.robot.commands.Command;

public class Controller {
    XboxController xbox = new XboxController(Constants.XBOX_PORT);
    GenericHID L_STICK = new GenericHID(Constants.JOYSTICK_L_PORT);
    GenericHID R_STICK = new GenericHID(Constants.JOYSTICK_R_PORT);

    IntakeHandler intake;
    ShooterHandler shooter;

    Curve sfr_curve;
    Curve sfl_curve;

    MPID fl_pid;
    MPID fr_pid;
    MPID bl_pid;
    MPID br_pid;

    public Controller() {
        this.sfr_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);
        this.sfl_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);

        this.fl_pid = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr_pid = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl_pid = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br_pid = new MPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);

        this.intake = new IntakeHandler();
        this.intake.idle();

        this.shooter = new ShooterHandler();
        this.shooter.idle();
    }

    double stickCurve(double val) {
        if (Math.abs(val) < 0.2) {
            val = 0;
        }
        return val;
    }

    double[] starControl(MainState state) {
        // left controllers give a polar vec, angle from 0 and mag from 0 to 1
        double[] stick_vec = { stickCurve(L_STICK.getRawAxis(0)), -1 * stickCurve(L_STICK.getRawAxis(1)) };
        SmartDashboard.putNumber("stick_L1", L_STICK.getRawAxis(1));
        SmartDashboard.putNumber("stick_L0", L_STICK.getRawAxis(0));
        SmartDashboard.putNumber("stick_R1", R_STICK.getRawAxis(1));
        SmartDashboard.putNumber("stick_R0", R_STICK.getRawAxis(0));
        double yaw_val = stickCurve(R_STICK.getRawAxis(0));
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

        double r2o2 = Math.sqrt(2) / 2;

        double max_vel = Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        double max_avl = max_vel / (Constants.ROBOT_WIDTH / 2);
        double rthe = SimpleMat.angleRectifier(theta + state.getHeadingVal());
        max_vel = max_vel * Math.max(Math.abs(Math.cos(rthe)), Math.abs(Math.sin(rthe)));
        double[] vel_vec = SimpleMat.projectHeading(theta, mag * max_vel * 0.1);
        SmartDashboard.putNumberArray("Vel Vec", vel_vec);

        double avel = max_avl * -1 * yaw_val * 0.01;
        SmartDashboard.putNumber("AVEL", avel);
        double[] whl_vec = MecanumIK.mecanumIK(vel_vec, avel);
        SmartDashboard.putNumberArray("MIKC", whl_vec);
        return whl_vec;
    }

    public Command getCommands(MainState state) {
        double lr_strafe = 0;
        double fb_1 = 0;
        double lr_turn = 0;
        double fb_2 = 0;
        double intake = 0;
        double storage = 0;
        double storage_2 = 0;
        double shooter_1 = 0;
        double shooter_2 = 0;

        if (DriverStation.isTeleop()) {

            lr_strafe = xbox.getLeftX() * 0.8;
            fb_1 = xbox.getLeftY();
            lr_turn = (xbox.getRightX());
            fb_2 = sfr_curve.getProp(xbox.getRightY());

            intake = xbox.getLeftTriggerAxis() * 0.4;// - stickCurve(this.R_STICK.getRawAxis(1));
            storage = xbox.getLeftTriggerAxis() * 0.4;

            storage_2 = 0;

            shooter_1 = xbox.getRightTriggerAxis();
            shooter_2 = xbox.getRightTriggerAxis();

            if (xbox.getLeftBumper() && this.intake.substate == 0) {
                intake = intake * -1;
                storage = storage * -1;
            }
            if (xbox.getRightBumper()) {
                storage_2 = 1;
            }
            if (xbox.getAButtonReleased()) {
                if (this.intake.substate == 0) {
                    this.intake.intakeStorage();
                } else {
                    this.intake.idle();
                }
            }
            if (xbox.getLeftStickButtonReleased()) {
                if (this.intake.substate == 0) {
                    this.intake.intakeStorage();
                } else {
                    this.intake.idle();
                }
            }
            if (xbox.getBButtonReleased()) {
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
            if (xbox.getYButton()) {
                storage_2 = -1;
            }
            double[] ins_cmd = this.intake.update(state);
            double[] sho_cmd = this.shooter.update(state);
            intake = ins_cmd[0] + sho_cmd[4];
            storage = ins_cmd[1] + sho_cmd[4];
            storage_2 = ins_cmd[2] + sho_cmd[0];
            shooter_1 = sho_cmd[1];
            shooter_2 = sho_cmd[2];
        }
        double[] whl_vec = starControl(state);

        double flt = -1 * fb_1 + lr_turn + lr_strafe;// + whl_vec[0];
        double frt = -1 * fb_1 - lr_turn - lr_strafe;// + whl_vec[1];
        double blt = -1 * fb_1 + lr_turn - lr_strafe;// + whl_vec[2];
        double brt = -1 * fb_1 - lr_turn + lr_strafe;// + whl_vec[3];

        flt = whl_vec[0] + (Math.min(1, Math.max(-1, flt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        frt = whl_vec[1] + (Math.min(1, Math.max(-1, frt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        blt = whl_vec[2] + (Math.min(1, Math.max(-1, blt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        brt = whl_vec[3] + (Math.min(1, Math.max(-1, brt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;

        double fld = flt - state.getFLRadssVal();
        double frd = frt - state.getFRRadssVal();
        double bld = blt - state.getBLRadssVal();
        double brd = brt - state.getBRRadssVal();

        double flr = this.fl_pid.update(fld);
        double frr = this.fr_pid.update(frd);
        double blr = this.bl_pid.update(bld);
        double brr = this.br_pid.update(brd);

        double[] ocmd = { flr, frr, blr, brr, intake, storage, storage_2, shooter_1, shooter_2 };
        Command command = new Command(ocmd);
        // Command command = new Command(flt*0.1, frt*0.1, blt*0.1,brt*0.1);
        return command;
    }
}