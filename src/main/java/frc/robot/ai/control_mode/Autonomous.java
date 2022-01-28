package frc.robot.ai.control_mode;

import frc.robot.ai.subroutines.*;
import frc.robot.commands.*;
import frc.robot.state.MainState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    enum Methods {
        TRAVEL,
        ROTATE
    }

    Methods[] cmdlist = { Methods.TRAVEL , Methods.TRAVEL, Methods.TRAVEL, Methods.TRAVEL, Methods.ROTATE};
    double[][] coords = { { 1, 1, Math.PI/4 } , {0,1,0}, {0,0,0}, {0,-1,-1*Math.PI/4}, {0, 0, 0}};
    int current_step = 0;

    // One for each method
    Travel travel;
    TurnToPoint rotate;

    public Autonomous() {
        this.current_step = 0;
    }

    public void init(MainState state) {
        this.current_step = 0;
        setMethods(state);
    }

    public void setMethods(MainState state) {
        switch (this.cmdlist[this.current_step]) {
            case TRAVEL:
                double[] pos = { coords[this.current_step][0], coords[this.current_step][1] };
                this.travel = new Travel(pos, coords[this.current_step][2], 1);
                this.travel.init(state);
                break;
            case ROTATE:
                double[] tpos = { coords[this.current_step][0], coords[this.current_step][1] };
                this.rotate = new TurnToPoint(tpos, 1);
                break;
        }
    }

    public Command getCommands(MainState state) {
        if (this.current_step + 1 > this.cmdlist.length) {
            return new Command(0, 0, 0, 0);
        }
        Command main_cmd = new Command(0, 0, 0, 0);
        boolean exit = false;
        SmartDashboard.putNumber("current step", this.current_step);
        switch (this.cmdlist[this.current_step]) {
            case TRAVEL:
                main_cmd = travel.update(state);
                exit = travel.exit(state);
                SmartDashboard.putNumber("Auton mcmd", main_cmd.bl_pprop);
                break;
            case ROTATE:
                main_cmd = rotate.update(state);
                exit = rotate.exit(state);
                break;
        }
        if (exit) {
            this.current_step++;
        }
        if (this.current_step + 1 > this.cmdlist.length) {
            return new Command(0, 0, 0, 0);
        } else {
            setMethods(state);
        }
        return main_cmd;
    }
}
