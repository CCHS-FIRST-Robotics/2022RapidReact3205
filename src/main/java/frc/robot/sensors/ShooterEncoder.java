package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareObjects;
import frc.robot.Constants;
import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;

public class ShooterEncoder extends BaseSensor {
    public ShooterEncoder(double sync_time) {
        this.SYNC_TIME = sync_time;
    }

    public void reset(HardwareObjects hardware) {
    }

    public boolean shouldUse() {
        return true;
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double storage_2 = hardware.STORAGE_2_MOTOR.getSelectedSensorVelocity(1);
        double shooter_1 = hardware.SHOOTER_1_MOTOR.getSelectedSensorVelocity(1) * -1;
        double shooter_2 = hardware.SHOOTER_2_MOTOR.getSelectedSensorVelocity(1) * -1;

        SmartDashboard.putNumber("ShooterEncoder/storage raw", storage_2);
        SmartDashboard.putNumber("ShooterEncoder/shooter 1 raw", shooter_1);
        SmartDashboard.putNumber("ShooterEncoder/shooter 2 raw", shooter_2);

        double dps2rads = 10 * 2 * Math.PI / 4096;

        storage_2 = storage_2 * dps2rads;
        shooter_1 = shooter_1 * dps2rads;
        shooter_2 = shooter_2 * dps2rads;

        state.setStorage2(storage_2, Constants.INIT_VARIANCE);
        state.setShooter(new double[] { shooter_1, shooter_2 }, Constants.INIT_VARIANCE);

        SmartDashboard.putNumber("ShooterEncoder/storage radss", storage_2);
        SmartDashboard.putNumber("ShooterEncoder/shooter 1 radss", shooter_1);
        SmartDashboard.putNumber("ShooterEncoder/shooter 2 radss", shooter_2);
    }
}
