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

public class Controller2 {
    XboxController xbox = new XboxController(Constants.XBOX_PORT);
    XboxController e_xbox = new XboxController(Constants.XBOX2_PORT);

    double flr = 0;
    double frr = 0;
    double blr = 0;
    double brr = 0;
    double in = 0;
    double storage = 0;
    double storage_2 = 0;
    double shooter_1 = 0;
    double shooter_2 = 0;
    double hangl = 0;
    double hangr = 0;

    IntakeHandler intake;
    ShooterHandler shooter;

    PointAtMiddle pam;
    FiringPosition arty;
    BallChase chase;

    int pam_s = 0;
    int arty_s = 0;
    int chase_s = 0;

    Curve sfr_curve;
    Curve sfl_curve;

    DPID fl_pid;
    DPID fr_pid;
    DPID bl_pid;
    DPID br_pid;

    double start_time = System.currentTimeMillis() / 1000;

    double pprop = 0.5;

    public Controller2() {
        this.sfr_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);
        this.sfl_curve = new Curve(Constants.SLOW_CURVE[0], Constants.SLOW_CURVE[1], Constants.SLOW_CURVE[2]);

        this.fl_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br_pid = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);

        this.intake = new IntakeHandler();
        this.intake.idle();

        this.shooter = new ShooterHandler();
        this.shooter.idle();
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

        double[] o_out = { 0, 0, 0, 0 };

        o_out[0] = this.fl_pid.updateRaw(whl_vec[0], state.getFLRadssVal());
        o_out[1] = this.fl_pid.updateRaw(whl_vec[1], state.getFRRadssVal());
        o_out[2] = this.fl_pid.updateRaw(whl_vec[2], state.getBLRadssVal());
        o_out[3] = this.fl_pid.updateRaw(whl_vec[3], state.getBRRadssVal());
        return o_out;
    }

    public void shooterStuff() {
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
    }

    public void intakeStuff(MainState state) {
        if (xbox.getRightBumper()) {
            if (this.intake.substate == 0) {
                this.intake.autoIntake(state);
            } else {
                this.intake.idle();
            }
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
                this.intake.intakeOnly();
            } else {
                this.intake.idle();
            }
        }
    }

    public double[] pamHandler(MainState state) {
        double[] pam_cmd = { 0, 0, 0, 0 };
        if (xbox.getRightStickButtonReleased() || e_xbox.getRightStickButtonReleased()) {
            if (this.pam_s == 0) {
                this.pam = new PointAtMiddle(state, 0.2);
                this.pam_s = 1;

                this.arty_s = 0;
                this.chase_s = 0;
            } else {
                this.pam_s = 0;
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
            }
        }
        return pam_cmd;
    }

    public double[] artyHandler(MainState state) {
        double[] arty_cmd = { 0, 0, 0, 0 };
        if (xbox.getLeftStickButtonReleased() || e_xbox.getLeftStickButtonReleased()) {
            if (this.arty_s == 0) {
                this.arty = new FiringPosition(state, 0.3);
                this.arty_s = 1;

                this.pam_s = 0;
                this.chase_s = 0;
            } else {
                this.arty_s = 0;
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
            }
        }
        return arty_cmd;
    }

    public double[] chaseHandler(MainState state, Map map) {
        double[] chase_cmd = { 0, 0, 0, 0 };
        if (xbox.getLeftBumperReleased()) {
            if (this.chase_s == 0) {
                this.intake.autoIntake(state);
                this.chase = new BallChase(state, map, -1, 0.3);
                this.chase_s = 1;

                this.arty_s = 0;
                this.pam_s = 0;
            } else {
                this.intake.idle();
                this.chase_s = 0;
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
                this.chase_s = this.chase_s;
            }
        }
        return chase_cmd;
    }

    public void rumbleHandler() {
        e_xbox.setRumble(RumbleType.kLeftRumble, 0.1);

        // rumble based on voltage , 1 at 7 and 0 at 11
        double voltage = RobotController.getBatteryVoltage();
        double rmb = 1 - ((voltage - 7) / 4);
        rmb = Math.min(Math.max(0, rmb), 1);
        xbox.setRumble(RumbleType.kRightRumble, rmb);
    }

    public void hangingHandler() {
        hangl = 0;
        hangr = 0;
        if (e_xbox.getAButton()) {
            hangl = 1;
            hangr = 1;
        }
        if (e_xbox.getBButton()) {
            hangl = -1;
            hangr = -1;
        }
    }

    public Command getCommands(MainState state, Map map, double CST) {

        this.pprop = 1 - xbox.getLeftTriggerAxis() * 0.25 - e_xbox.getLeftTriggerAxis() * 0.25
                - e_xbox.getRightTriggerAxis() * 0.25;
        double[] wv_co = starControl(state);
        double[] wv_ch = chaseHandler(state, map);
        double[] wv_ar = artyHandler(state);
        double[] wv_pm = pamHandler(state);
        flr = wv_co[0] + wv_ch[0] + wv_ar[0] + wv_pm[0];
        frr = wv_co[1] + wv_ch[1] + wv_ar[1] + wv_pm[1];
        blr = wv_co[2] + wv_ch[2] + wv_ar[2] + wv_pm[2];
        brr = wv_co[3] + wv_ch[3] + wv_ar[3] + wv_pm[3];

        in = e_xbox.getRightY() - xbox.getRightTriggerAxis();
        storage = e_xbox.getRightX() - xbox.getRightTriggerAxis();
        storage_2 = e_xbox.getLeftX();
        shooter_1 = e_xbox.getLeftY() * -1;
        shooter_2 = e_xbox.getLeftY() * -1;
        intakeStuff(state);
        shooterStuff();
        double[] ins_cmd = this.intake.update(state);
        double[] sho_cmd = this.shooter.update(state);
        if (this.intake.substate != 0) {
            in = in + ins_cmd[0];
            storage = storage + ins_cmd[1];
            storage_2 = storage_2 + ins_cmd[2] + sho_cmd[0];
        }
        if (this.shooter.state != 0) {
            in = in + ins_cmd[0] + sho_cmd[3];
            storage = storage + ins_cmd[1] + sho_cmd[3];
            storage_2 = storage_2 + ins_cmd[2] + sho_cmd[0];
            shooter_1 = shooter_1 + sho_cmd[1];
            shooter_2 = shooter_2 + sho_cmd[2];
        }
        hangingHandler();
        rumbleHandler();
        double[] ocmd = { flr, frr, blr, brr, in, storage, storage_2, shooter_1, shooter_2, hangl, hangr };
        Command command = new Command(ocmd);
        return command;
    }
}
