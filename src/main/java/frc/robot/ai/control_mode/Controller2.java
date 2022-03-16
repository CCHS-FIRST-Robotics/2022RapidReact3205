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
        return whl_vec;
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

    public Command getCommand(MainState state, Map map, double CST) {

        this.pprop = 0.5 + xbox.getLeftTriggerAxis() * 0.5;
        double[] whl_vec = starControl(state);
        double[] ocmd = { flr, frr, blr, brr, in, storage, storage_2, shooter_1, shooter_2 };
        Command command = new Command(ocmd);
        return command;
    }
}
