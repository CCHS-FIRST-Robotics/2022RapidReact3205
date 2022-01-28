package frc.robot.ai.control_mode;

import frc.robot.ai.subroutines.*;
import frc.robot.commands.*;
import frc.robot.state.MainState;

public class Autonomous {
    enum Methods {
        TRAVEL
    }

    Methods[] cmdlist = { Methods.TRAVEL };
    double[][] coords = { { 0, 0, 0 } };
    int current_step = 0;

    // One for each method
    Travel travel;

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
                this.travel = new Travel(pos, coords[this.current_step][2], 0.7);
                this.travel.init(state);
        }
    }

    public Command getCommands(MainState state) {
        if (this.current_step + 1 > this.cmdlist.length) {
            return new Command(0, 0, 0, 0);
        }
        Command main_cmd = new Command(0, 0, 0, 0);
        boolean exit = false;
        switch (this.cmdlist[this.current_step]) {
            case TRAVEL:
                main_cmd = travel.update(state);
                exit = travel.exit(state);
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
