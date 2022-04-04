package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Rintake {
    public Rintake(){

    }
    public void setDrives(HardwareObjects hardware, double pwr) {
        hardware.RINTAKE_MOTOR.set( -1 * pwr);
    }
}
