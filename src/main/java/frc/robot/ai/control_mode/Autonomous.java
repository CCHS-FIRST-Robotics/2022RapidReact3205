package frc.robot.ai.control_mode;

import frc.robot.Constants;
import frc.robot.ai.subroutines.*;
import frc.robot.commands.*;
import frc.robot.state.MainState;
import frc.robot.ai.routines.Methods;
import frc.robot.ai.routines.TestAuton;

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

    ArrayList<Methods> cmd_list = TestAuton.sinCurveMethods();
    ArrayList<Double> ang_list = TestAuton.sinCurveAngs();
    ArrayList<ArrayList<double[]>> coord_list = TestAuton.sinCurveVals();

    int current_step = 0;

    // One for each method
    SimpleTravel travel;
    TurnToPoint rotate;
    CurveFwdTravel curve;

    public Autonomous() {
        this.current_step = 0;
    }

    public void init(MainState state) {
        this.current_step = 0;
        setMethods(state);
    }

    public void setMethods(MainState state) {
        switch (this.cmd_list.get(this.current_step)) {
            case TRAVEL:
                double[] pos = { this.coord_list.get(this.current_step).get(0)[0],
                        this.coord_list.get(this.current_step).get(0)[1] };
                this.travel = new SimpleTravel(pos, this.ang_list.get(this.current_step), 1);
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
                        this.ang_list.get(this.current_step), 1);
        }
    }

    public Command getCommands(MainState state) {
        if (this.current_step + 1 > this.cmd_list.size()) {
            return new Command(Constants.DEFAULT_CMD);
        }
        Command main_cmd = new Command(Constants.DEFAULT_CMD);
        boolean exit = false;
        switch (this.cmd_list.get(this.current_step)) {
            case TRAVEL:
                main_cmd = this.travel.update(state);
                exit = this.travel.exit(state);
                SmartDashboard.putNumber("Auton mcmd", main_cmd.bl_pprop);
                break;
            case ROTATE:
                main_cmd = this.rotate.update(state);
                exit = this.rotate.exit(state);
                break;
            case CURVE:
                main_cmd = this.curve.update(state);
                exit = this.curve.exit(state);
        }
        if (exit) {
            this.current_step++;
        }
        if (this.current_step + 1 > this.cmd_list.size()) {
            return new Command(Constants.DEFAULT_CMD);
        } else {
            setMethods(state);
        }
        return main_cmd;
    }
}
