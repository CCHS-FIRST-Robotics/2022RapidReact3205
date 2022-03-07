package frc.robot.ai.control_mode;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.ai.subroutines.*;
import frc.robot.commands.*;
import frc.robot.state.MainState;
import frc.robot.ai.routines.*;
import frc.robot.map.Map;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

    // Methods[] cmdlist = { Methods.TRAVEL, Methods.TRAVEL, Methods.TRAVEL,
    // Methods.TRAVEL, Methods.ROTATE,
    // Methods.TRAVEL, Methods.TRAVEL, Methods.ROTATE, Methods.TRAVEL,
    // Methods.TRAVEL, Methods.ROTATE,
    // Methods.TRAVEL, Methods.TRAVEL };
    // double[][] coords = { { 1, 1, Math.PI / 4 }, { 0, 1, 0 }, { 0, 0, 0 }, { 0,
    // -1, -1 * Math.PI / 4 }, { 0, 0, 0 },
    // { -0.5, 1.5, Math.PI / 4 }, { -0.5, 1, 0 }, { 5, 3, 0 }, { 0.4, -0.4, -0.75 *
    // 3.14 },
    // { 0.7, -0.7, -0.75 * 3.14 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    SR1_R1 generator;

    double[] start_pos;
    double start_heading;
    ArrayList<Methods> cmd_list;
    ArrayList<Double> ang_list;
    ArrayList<ArrayList<double[]>> coord_list;

    int current_step = 0;

    // One for each method
    SimpleTravel travel;
    TurnToPoint rotate;
    CurveFwdTravel curve;
    PointAtMiddle pam;
    FiringPosition firing_pos;

    // General Hardware methods
    IntakeHandler intake;
    ShooterHandler shooter;

    public Autonomous() {
        this.current_step = 0;
        this.intake = new IntakeHandler();
        this.shooter = new ShooterHandler();
    }

    public void init(HardwareObjects hardware, MainState state, Map map) {
        this.current_step = -1;
        this.generator = new SR1_R1();

        this.start_pos = this.generator.start_point;
        this.start_heading = this.generator.start_heading;

        this.cmd_list = this.generator.met_list;
        this.ang_list = this.generator.angsl;
        this.coord_list = this.generator.vals;

        map.softInit(state, this.start_pos, this.start_heading);
        map.pos.start_pos = this.start_pos;
        map.pos.heading = this.start_heading;
        map.initialize(hardware);
        this.current_step = 0;
        setMethods(state);
    }

    public void setMethods(MainState state) {
        switch (this.cmd_list.get(this.current_step)) {
            case TRAVEL:
                double[] pos = { this.coord_list.get(this.current_step).get(0)[0],
                        this.coord_list.get(this.current_step).get(0)[1] };
                this.travel = new SimpleTravel(pos, this.ang_list.get(this.current_step), 0.4);
                this.travel.init(state);
                SmartDashboard.putString("Auton/func", "TRAVEL");
                break;
            case ROTATE:
                double[] tpos = { this.coord_list.get(this.current_step).get(0)[0],
                        this.coord_list.get(this.current_step).get(0)[1] };
                this.rotate = new TurnToPoint(tpos, 1);
                SmartDashboard.putString("Auton/func", "ROTATE");
                break;
            case CURVE:
                this.curve = new CurveFwdTravel(this.coord_list.get(this.current_step),
                        this.ang_list.get(this.current_step), 0.4);
                this.curve.init(state);
                SmartDashboard.putString("Auton/func", "CURVE");
                break;
            case POINT_MID:
                this.pam = new PointAtMiddle(state, 1);
                this.pam.init(state);
                SmartDashboard.putString("Auton/func", "PAM");
                break;
            case FIRE_MID:
                this.firing_pos = new FiringPosition(state, 0.2);
                this.firing_pos.init(state);
                SmartDashboard.putString("Auton/func", "FIRE MID");
                break;
            case INTAKE_ONLY:
                this.intake.intakeOnly();
                SmartDashboard.putString("Auton/func", "INTAKE_ONLY");
                break;
            case INTAKE_STORE:
                this.intake.intakeStorage();
                SmartDashboard.putString("Auton/func", "INTAKE_STORE");
                break;
            case INTAKE_IDLE:
                this.intake.idle();
                SmartDashboard.putString("Auton/func", "INTAKE_IDLE");
                break;
            case SHOOTER_FIRE:
                this.shooter.initFiring();
                SmartDashboard.putString("Auton/func", "SHOOTER_FIRE");
                break;
        }
    }

    public Command getCommands(MainState state) {
        if (this.current_step == -1){
            return new Command(Constants.DEFAULT_CMD);
        }
        if (this.current_step + 1 > this.cmd_list.size()) {
            return new Command(Constants.DEFAULT_CMD);
        }
        Command main_cmd = new Command(Constants.DEFAULT_CMD);
        boolean exit = false;
        switch (this.cmd_list.get(this.current_step)) {
            case TRAVEL:
                main_cmd = this.travel.update(state);
                exit = this.travel.exit(state);
                SmartDashboard.putString("Auton/func_exe", "TRAVEL");
                SmartDashboard.putNumber("Auton/travel cmd", main_cmd.fl_pprop);
                SmartDashboard.putNumberArray("Auton/travel point", this.travel.tpos);
                break;
            case ROTATE:
                main_cmd = this.rotate.update(state);
                exit = this.rotate.exit(state);
                SmartDashboard.putString("Auton/func_exe", "ROTATE");
                break;
            case CURVE:
                main_cmd = this.curve.update(state);
                exit = this.curve.exit(state);
                SmartDashboard.putString("Auton/func_exe", "CURVE");
                break;
            case POINT_MID:
                main_cmd = this.pam.update(state);
                exit = this.pam.exit(state);
                SmartDashboard.putString("Auton/func_exe", "POINT_MID");
                break;
            case FIRE_MID:
                main_cmd = this.firing_pos.update(state);
                exit = this.firing_pos.exit(state);
                SmartDashboard.putString("Auton/func_exe", "FIRE_MID");
                break;
            case INTAKE_ONLY:
                exit = true;
                SmartDashboard.putString("Auton/func_exe", "INTAKE_ONLY");
                break;
            case INTAKE_STORE:
                exit = intake.exit();
                SmartDashboard.putString("Auton/func_exe", "INTAKE_STORE");
                break;
            case INTAKE_IDLE:
                this.intake.idle();
                exit = true;
                SmartDashboard.putString("Auton/func_exe", "INTAKE_IDLE");
                break;
            case SHOOTER_FIRE:
                exit = shooter.exit();
                SmartDashboard.putString("Auton/func_exe", "SHOOTER_FIRE");
                break;
        }
        if (exit) {
            this.current_step++;
            setMethods(state);
        }
        double[] intake_cmd = this.intake.update(state);
        double[] shooter_cmd = this.shooter.update(state);
        SmartDashboard.putNumberArray("Auton/intake_cmd", intake_cmd);
        SmartDashboard.putNumberArray("Auton/shooter_cmd", intake_cmd);
        SmartDashboard.putNumber("Auton/FL",main_cmd.fr_pprop);
        SmartDashboard.putNumber("Auton/step", this.current_step);
        main_cmd.intake_pprop = intake_cmd[0];
        main_cmd.storage_1_pprop = intake_cmd[1];
        main_cmd.storage_2_pprop = shooter_cmd[0] + intake_cmd[2];
        main_cmd.shooter1_pprop = shooter_cmd[1];
        main_cmd.shooter2_pprop = shooter_cmd[2];

        if (this.current_step + 1 > this.cmd_list.size()) {
            return new Command(Constants.DEFAULT_CMD);
        }
        return main_cmd;
    }
}
