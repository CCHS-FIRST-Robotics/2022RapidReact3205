package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.commands.subsystems.*;
import edu.wpi.first.wpilibj.RobotController;

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
    private Intake intake = new Intake();
    private Shooter shooter = new Shooter();
    public Hanging hanging = new Hanging();

    /**
     * Constructor for CommandHandler
     */
    public CommandHandler() {
        this.drive = new Drive();
        this.intake = new Intake();
        this.hanging = new Hanging();
    }

    /**
     * schedule commands using a command. Takes every hardware interface object such
     * as drive and sets values.
     * 
     * @param command  command issued to the robot.
     * @param hardware robot hardware objects.
     */
    public void scheduleCommands(Command command, HardwareObjects hardware) {
        // Brownout protection
        double voltage = RobotController.getBatteryVoltage();
        double concern_const = 1;
        if (voltage < Constants.VOLT_CONCERN_RANGE[0]) {
            concern_const = 0;
        } else if (voltage < Constants.VOLT_CONCERN_RANGE[1]) {
            concern_const = (voltage - Constants.VOLT_CONCERN_RANGE[0])
                    / (Constants.VOLT_CONCERN_RANGE[1] - Constants.VOLT_CONCERN_RANGE[0]);
        }

        command.fl_pprop = command.fl_pprop * concern_const;
        command.fr_pprop = command.fr_pprop * concern_const;
        command.bl_pprop = command.bl_pprop * concern_const;
        command.br_pprop = command.br_pprop * concern_const;

        if (Math.abs(command.storage_2_pprop) < 0.1){
            command.storage_2_pprop = -0.03;
        }

        // Schedule hardware commands using command
        this.drive.setDrives(command.fl_pprop, command.fr_pprop, command.bl_pprop, command.br_pprop, hardware);
        this.intake.setDrives(command.intake_pprop, command.storage_1_pprop, hardware);
        this.shooter.setDrives(command.storage_2_pprop, command.shooter1_pprop, command.shooter2_pprop, hardware);
        this.hanging.setDrives(command.hang_l_pprop, command.hang_r_pprop, hardware);
    }
}