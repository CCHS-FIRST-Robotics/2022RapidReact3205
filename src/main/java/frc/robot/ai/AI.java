package frc.robot.ai;

import frc.robot.state.*;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.ai.control_mode.*;
import frc.robot.commands.*;
import frc.robot.map.Map;
import frc.robot.network.Network;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public Command getCommand(HardwareObjects hardware, MainState state, Map map, double CST, Network net1) {
        // SmartDashboard.putBoolean("Doing Disabled", false);
        switch (this.current_state) {
            case CONTROLLER:
                main_command = this.controller_state.getCommands(state, map, CST);
                break;
            case AUTONOMOUS:
                main_command = this.autonomous.getCommands(state, map, hardware, net1);
                break;
            case DISABLED:
                Constants.START_POS = new double[] { Constants.START[0], Constants.START[1] };
                map.softInit(hardware, state, Constants.START_POS, Constants.START_H);
                // state.setPos(Constants.sp, 0.01);
                // SmartDashboard.putBoolean("Doing Disabled", true);
                // SmartDashboard.putNumberArray("SP", Constants.sp);
                main_command = new Command(Constants.DEFAULT_CMD);
                break;
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

    public void setAutonomousState(HardwareObjects hardware, MainState state, Map map, Network net) {
        // this.autonomous.init(hardware, state, map, net);
        this.current_state = States.AUTONOMOUS;
    }

    public void setDisabledState() {
        this.current_state = States.DISABLED;
    }
}