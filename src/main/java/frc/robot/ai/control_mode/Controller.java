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

    IntakeHandler intake;

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
    }

    public Command getCommands(MainState state) {
        double lr_strafe = xbox.getLeftX();
        double fb_1 = xbox.getLeftY();
        double lr_turn = (xbox.getRightX());
        double fb_2 = sfr_curve.getProp(xbox.getRightY());

        double intake = xbox.getLeftTriggerAxis();
        double storage = xbox.getRightTriggerAxis();
        if (xbox.getLeftBumper() && this.intake.substate == 0) {
            intake = intake * -1;
        }
        if (xbox.getRightBumper() && this.intake.substate == 0) {
            storage = storage * -1;
        }
        if (xbox.getAButtonReleased()) {
            this.intake.intakeOnly();
        }
        if (xbox.getBButtonReleased()) {
            this.intake.intakeStorage();
        }
        if (xbox.getXButtonReleased()) {
            this.intake.idle();
        }
        if (this.intake.substate != 0) {
            double[] ins_cmd = this.intake.update(state);
            intake = ins_cmd[0];
            storage = ins_cmd[1];
        }

        double flt = -1 * fb_1 - fb_2 + lr_turn + lr_strafe;
        double frt = -1 * fb_1 - fb_2 - lr_turn - lr_strafe;
        double blt = -1 * fb_1 - fb_2 + lr_turn - lr_strafe;
        double brt = -1 * fb_1 - fb_2 - lr_turn + lr_strafe;

        flt = Math.min(1, Math.max(-1, flt)) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        frt = Math.min(1, Math.max(-1, frt)) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        blt = Math.min(1, Math.max(-1, blt)) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        brt = Math.min(1, Math.max(-1, brt)) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;

        double fld = flt - state.getFLRadssVal();
        double frd = frt - state.getFRRadssVal();
        double bld = blt - state.getBLRadssVal();
        double brd = brt - state.getBRRadssVal();

        double flr = this.fl_pid.update(fld);
        double frr = this.fr_pid.update(frd);
        double blr = this.bl_pid.update(bld);
        double brr = this.br_pid.update(brd);

        double[] ocmd = { flr, frr, blr, brr, intake, storage };
        Command command = new Command(ocmd);
        // Command command = new Command(flt*0.1, frt*0.1, blt*0.1,brt*0.1);
        return command;
    }
}
