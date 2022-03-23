package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Hanging {
    public Hanging() {

    }

    public void setDrives(double hang_l, double hang_r, HardwareObjects hardware) {
        hardware.HANG_L_MOTOR.set(ControlMode.PercentOutput, hang_l);
        hardware.HANG_R_MOTOR.set(ControlMode.PercentOutput, hang_r * -1);
    }
}
