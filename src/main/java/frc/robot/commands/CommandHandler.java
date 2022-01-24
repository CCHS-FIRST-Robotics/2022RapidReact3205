package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.commands.subsystems.Drive;

import java.lang.Math;
import frc.robot.helper.SimpleMat;
import frc.robot.HardwareObjects;

/**
 * Command Handler class that takes command object and uses it to schedule
 * commands to hardware objects.
 */
public class CommandHandler {
    // Define devices as atrributes
    private Drive drive = new Drive();

    /**
     * Constructor for CommandHandler
     */
    public CommandHandler() {
        this.drive = new Drive();
    }

    /**
     * schedule commands using a command. Takes every hardware interface object such
     * as drive and sets values.
     * 
     * @param command  command issued to the robot.
     * @param hardware robot hardware objects.
     */
    public void scheduleCommands(Command command, HardwareObjects hardware) {
        // Schedule hardware commands using command
        this.drive.setDrives(command.fl_pprop, command.fr_pprop, command.bl_pprop, command.br_pprop, hardware);
    }
}