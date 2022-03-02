package frc.robot.ai.control_mode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
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

    PID fl_pid;
    PID fr_pid;
    PID bl_pid;
    PID br_pid;

    public Controller() {
        this.sfr_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);
        this.sfl_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);

        this.fl_pid = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr_pid = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl_pid = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br_pid = new PID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);

        this.intake = new IntakeHandler();
        this.intake.idle();

        this.shooter = new ShooterHandler();
        this.shooter.idle();
    }

    double[] starControl(MainState state) {
        // left controllers give a polar vec, angle from 0 and mag from 0 to 1
        double[] stick_vec = { L_STICK.getRawAxis(1), -1 * L_STICK.getRawAxis(2) };
        double yaw_val = R_STICK.getRawAxis(1);
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
        double rthe = SimpleMat.angleRectifier(theta - state.getHeadingVal());
        max_vel = max_vel * Math.max(Math.abs(Math.cos(rthe)), Math.abs(Math.sin(rthe)));
        double[] vel_vec = SimpleMat.projectHeading(theta, mag * max_vel);
        double avel = max_avl * -1 * yaw_val;

        double[] whl_vec = MecanumIK.mecanumIK(vel_vec, avel);
        return whl_vec;
    }

    public Command getCommands(MainState state) {
        double lr_strafe = xbox.getLeftX();
        double fb_1 = xbox.getLeftY();
        double lr_turn = (xbox.getRightX());
        double fb_2 = sfr_curve.getProp(xbox.getRightY());

        double intake = xbox.getLeftTriggerAxis() + this.R_STICK.getRawAxis(2);
        double storage = xbox.getLeftTriggerAxis();

        double storage_2 = 0;

        double shooter_1 = xbox.getRightTriggerAxis();
        double shooter_2 = xbox.getRightTriggerAxis();

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
        if (xbox.getBButtonReleased()) {
            if (this.intake.substate == 0) {
                this.intake.intakeStorage();
            } else {
                this.intake.idle();
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

        if (this.intake.substate != 0) {
            double[] ins_cmd = this.intake.update(state);
            intake = ins_cmd[0];
            storage = ins_cmd[1];
        }
        double[] sho_cmd = this.shooter.update(state);
        if (this.shooter.state != 0) {
            storage_2 = sho_cmd[0];
            shooter_1 = sho_cmd[1];
            shooter_2 = sho_cmd[2];
        }

        double[] whl_vec = starControl(state);

        double flt = -1 * fb_1 + lr_turn + lr_strafe + whl_vec[0];
        double frt = -1 * fb_1 - lr_turn - lr_strafe + whl_vec[1];
        double blt = -1 * fb_1 + lr_turn - lr_strafe + whl_vec[2];
        double brt = -1 * fb_1 - lr_turn + lr_strafe + whl_vec[3];

        flt = (Math.min(1, Math.max(-1, flt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        frt = (Math.min(1, Math.max(-1, frt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        blt = (Math.min(1, Math.max(-1, blt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        brt = (Math.min(1, Math.max(-1, brt)) - fb_2) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;

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