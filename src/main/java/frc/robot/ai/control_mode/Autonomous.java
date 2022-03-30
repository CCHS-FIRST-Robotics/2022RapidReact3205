package frc.robot.ai.control_mode;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.ai.subroutines.*;
import frc.robot.commands.*;
import frc.robot.helper.*;
import frc.robot.state.MainState;
import frc.robot.ai.routines.*;
import frc.robot.map.Map;
import frc.robot.network.Network;

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

    Auto15_2 generator;

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
    BallChase ball_chase;
    RawTranslate rtrans;
    double wait_t = 0;
    double end_time = 0;

    // General Hardware methods
    IntakeHandler intake;
    ShooterHandler shooter;
    public Boolean initted = false;

    public Autonomous() {
        this.current_step = 0;
        this.intake = new IntakeHandler();
        this.shooter = new ShooterHandler();
        this.initted = false;

    }

    public void init(HardwareObjects hardware, MainState state, Map map, Network net) {
        this.current_step = -1;
        this.generator = new Auto15_2();

        this.start_pos = this.generator.start_point;
        this.start_heading = this.generator.start_heading;

        this.cmd_list = this.generator.met_list;
        this.ang_list = this.generator.angsl;
        this.coord_list = this.generator.vals;

        map.softInit(hardware, state, this.start_pos, this.start_heading);
        // Attempt Limelight localization
        if (net.lime.getValid()) {
            double[] lime_pos = LimeHelper.getPos(state, net);
            this.start_pos = lime_pos;
            
        }
        map.softInit(hardware, state, this.start_pos, this.start_heading);
        map.pos.start_pos = this.start_pos;
        map.pos.heading = this.start_heading;
        map.initialize(hardware);
        this.current_step = 0;
        setMethods(state, map);
        this.initted = true;
    }

    public void setMethods(MainState state, Map map) {
        
        switch (this.cmd_list.get(this.current_step)) {
            case TRAVEL:
                double[] pos = { this.coord_list.get(this.current_step).get(0)[0],
                        this.coord_list.get(this.current_step).get(0)[1] };
                this.travel = new SimpleTravel(pos, this.ang_list.get(this.current_step), 0.3);
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
                        this.ang_list.get(this.current_step), 0.5);
                this.curve.init(state);
                SmartDashboard.putString("Auton/func", "CURVE");
                break;
            case POINT_MID:
                this.pam = new PointAtMiddle(state, 0.2);
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
            case SHOOTER_DOUBLE:
                this.shooter.initDoubleFiring();
                SmartDashboard.putString("Auton/func", "SHOOTER_DOUBLE");
                break;
            case BALL_CHASE:
                this.intake.autoIntake(state);
                this.ball_chase = new BallChase(state, map, -1, 0.3);
                SmartDashboard.putString("Auton/func", "BALL CHASE");
                break;
            case DROP_INTAKE:
                this.rtrans = new RawTranslate(new double[] {0, -0.3}, 0.2);
                SmartDashboard.putString("Auton/func", "DROP");
                break;
            case INTAKE_STATIC:
                this.intake.autoIntake(state);
                SmartDashboard.putString("Auton/func", "INTAKE_STATIC");
                break;
            case WAIT:
                this.end_time = this.ang_list.get(this.current_step);
                this.wait_t = System.currentTimeMillis() / 1000;
                SmartDashboard.putString("Auton/func", "WAIT");
                break;
            case RAW_TRANS:
                double[] move_vec = { this.coord_list.get(this.current_step).get(0)[0],
                    this.coord_list.get(this.current_step).get(0)[1] };
                this.rtrans = new RawTranslate(move_vec, this.ang_list.get(this.current_step));
                SmartDashboard.putString("Auton/func", "Raw Trans");
                break;
        }
    }

    public Command getCommands(MainState state, Map map, HardwareObjects hardware, Network net) {
        if (this.initted == false){
            init(hardware, state, map, net);
        }
        if (this.current_step == -1) {
            return new Command(Constants.DEFAULT_CMD);
        }
        if (this.current_step + 1 > this.cmd_list.size()) {
            this.current_step = -1;
            return new Command(Constants.DEFAULT_CMD);
        }
        Command main_cmd = new Command(Constants.DEFAULT_CMD);
        boolean exit = false;
        switch (this.cmd_list.get(this.current_step)) {
            case TRAVEL:
                main_cmd = this.travel.update(state);
                exit = this.travel.exit(state);
                //SmartDashboard.putString("Auton/func_exe", "TRAVEL");
                //SmartDashboard.putNumber("Auton/travel cmd", main_cmd.fl_pprop);
                //SmartDashboard.putNumberArray("Auton/travel point", this.travel.tpos);
                break;
            case ROTATE:
                main_cmd = this.rotate.update(state);
                exit = this.rotate.exit(state);
                //SmartDashboard.putString("Auton/func_exe", "ROTATE");
                break;
            case CURVE:
                main_cmd = this.curve.update(state);
                exit = this.curve.exit(state);
                //SmartDashboard.putString("Auton/func_exe", "CURVE");
                break;
            case POINT_MID:
                main_cmd = this.pam.update(state);
                exit = this.pam.exit(state);
                //SmartDashboard.putString("Auton/func_exe", "POINT_MID");
                break;
            case FIRE_MID:
                main_cmd = this.firing_pos.update(state);
                exit = this.firing_pos.exit(state);
                //SmartDashboard.putString("Auton/func_exe", "FIRE_MID");
                break;
            case INTAKE_ONLY:
                exit = true;
                //SmartDashboard.putString("Auton/func_exe", "INTAKE_ONLY");
                break;
            case INTAKE_STORE:
                exit = true;
                //SmartDashboard.putString("Auton/func_exe", "INTAKE_STORE");
                break;
            case INTAKE_IDLE:
                this.intake.idle();
                exit = true;
                //SmartDashboard.putString("Auton/func_exe", "INTAKE_IDLE");
                break;
            case SHOOTER_FIRE:
                exit = shooter.exit();
                //SmartDashboard.putString("Auton/func_exe", "SHOOTER_FIRE");
                break;
            case SHOOTER_DOUBLE:
                main_cmd = new Command(Constants.DEFAULT_CMD);
                exit = shooter.exit();
                //SmartDashboard.putString("Auton/func_exe", "SHOOTER_DOUBLE");
                break;
            case BALL_CHASE:
                main_cmd = this.ball_chase.update(state, map);
                main_cmd = new Command(Constants.DEFAULT_CMD);
                exit = this.ball_chase.exit(state, map);
                //SmartDashboard.putString("Auton/func_exe", "BALL CHASE");
                break;
            case DROP_INTAKE:
                main_cmd = this.rtrans.update(state);
                exit = this.rtrans.exit();
                break;
            case INTAKE_STATIC:
                exit = this.intake.exit();
                break;
            case WAIT:
                if (System.currentTimeMillis() / 1000 - this.wait_t > this.end_time) {
                    exit = true;
                }
                break;
            case RAW_TRANS:
                main_cmd = this.rtrans.update(state);
                exit = this.rtrans.exit();
                break;
        }
        if (exit) {
            this.current_step++;
            if (this.current_step + 1 > this.cmd_list.size()) {
                return new Command(Constants.DEFAULT_CMD);
            }
            setMethods(state, map);
        }
        double[] intake_cmd = this.intake.update(state);
        double[] shooter_cmd = this.shooter.update(state);
        //SmartDashboard.putNumberArray("Auton/intake_cmd", intake_cmd);
        //SmartDashboard.putNumberArray("Auton/shooter_cmd", intake_cmd);
        //SmartDashboard.putNumber("Auton/FL", main_cmd.fr_pprop);
        //SmartDashboard.putNumber("Auton/step", this.current_step);
        main_cmd.intake_pprop = intake_cmd[0] + shooter_cmd[3];
        main_cmd.storage_1_pprop = intake_cmd[1] + shooter_cmd[3];
        main_cmd.storage_2_pprop = shooter_cmd[0] + intake_cmd[2];
        main_cmd.shooter1_pprop = shooter_cmd[1];
        main_cmd.shooter2_pprop = shooter_cmd[2];
        return main_cmd;
    }
}
