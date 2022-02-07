package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Intake {
    public Intake() {

    }

    public void setDrives(double intake, double storage, HardwareObjects hardware) {
        hardware.INTAKE_MOTOR.set(ControlMode.PercentOutput, intake * -1);
        hardware.STORAGE_1_MOTOR.set(ControlMode.PercentOutput, storage);
    }
}
