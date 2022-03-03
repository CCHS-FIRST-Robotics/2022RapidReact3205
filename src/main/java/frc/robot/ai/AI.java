package frc.robot.ai;

import frc.robot.state.*;
import frc.robot.Constants;
import frc.robot.ai.control_mode.*;
import frc.robot.commands.*;
import frc.robot.map.Map;

/**
 * AI finite state machine class, handles switching between various autonomous
 * and controller states. Should eventually gain capability to reset states and
 * load various autons
 */
public class AI {
    enum States {
        CONTROLLER, AUTONOMOUS, DISABLED
    }

    public States current_state = States.CONTROLLER;
    public Controller controller_state = new Controller();
    public Command main_command = new Command(Constants.DEFAULT_CMD);
    // public AutonomousTestDrive autonomousTestDrive = new AutonomousTestDrive();
    // public AutonomousTravel autonomous;
    public Autonomous autonomous;

    /**
     * Constructor for AI
     */
    public AI() {
        this.current_state = States.CONTROLLER;
        this.controller_state = new Controller();
        this.main_command = new Command(Constants.DEFAULT_CMD);
        this.autonomous = new Autonomous();
    }

    /**
     * Get hardware command from the various states
     * 
     * @param state main robot state
     * @return hardware command
     */
    public Command getCommand(MainState state) {
        switch (this.current_state) {
            case CONTROLLER:
                main_command = this.controller_state.getCommands(state);
                break;
            case AUTONOMOUS:
                main_command = this.autonomous.getCommands(state);
                break;
            case DISABLED:
                main_command = new Command(Constants.DEFAULT_CMD);
        }
        return main_command;
    }

    public String getFiniteState() {
        if (this.current_state == States.CONTROLLER) {
            return "CONTROLLER";
        } else if (this.current_state == States.AUTONOMOUS) {
            return "AUTONOMOUS";
        } else {
            return "N/A";
        }
    }

    public void setControllerState() {
        this.current_state = States.CONTROLLER;
    }

    public void setAutonomousState(MainState state, Map map) {
        this.autonomous.init(state, map);
        this.current_state = States.AUTONOMOUS;
    }
}