package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Drive {
    public Drive() {

    }

    public void setDrives(double fl, double fr, double bl, double br, HardwareObjects hardware) {
        hardware.FLD_MOTOR.set(ControlMode.PercentOutput, fl);
        hardware.FRD_MOTOR.set(ControlMode.PercentOutput, fr * -1);
        hardware.BLD_MOTOR.set(ControlMode.PercentOutput, bl * -1);
        hardware.BRD_MOTOR.set(ControlMode.PercentOutput, br);
    }
}
