package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    public Shooter() {

    }

    public void setDrives(double storage_2, double shooter1, double shooter2, HardwareObjects hardware) {
        hardware.STORAGE_2_MOTOR.set(ControlMode.PercentOutput, storage_2 * -1);
        hardware.SHOOTER_1_MOTOR.set(ControlMode.PercentOutput, shooter1 * -1);
        hardware.SHOOTER_2_MOTOR.set(ControlMode.PercentOutput, shooter2 * -1);
        SmartDashboard.putNumber("Shooter/Storage V", hardware.STORAGE_2_MOTOR.getMotorOutputVoltage());
        SmartDashboard.putNumber("Shooter/Storage P", hardware.STORAGE_2_MOTOR.getMotorOutputPercent());
    }
}
