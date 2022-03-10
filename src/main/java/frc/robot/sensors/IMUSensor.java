package frc.robot.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;
import frc.robot.helper.*;

/**
 * Sensor handling IMU values, main way of updating heading and also updates vel
 * and projected acceleration
 * 
 * @author Ludwig Tay
 */
public class IMUSensor extends BaseSensor {

    double ang_var = 0.0;

    public boolean log_active_sensor;
    public double log_fused_heading;
    public double[] log_acc = { 0, 0 };
    public double log_pitch;
    public double log_yaw_vel;

    double x_acc_zero = 0;
    double[] xyz_acc_zero = { 0, 0, 9.81 };
    double yz_mag_zero = 9.81;

    /**
     * Constructor for IMUSensor.
     * 
     * @param sync_time UNUSED.
     */
    public IMUSensor(double sync_time) {
        this.ang_var = Constants.BASE_HEADING_VAR;
        this.SYNC_TIME = sync_time;
    }

    /**
     * Standard sensor method to determine whether to use or not in main loop.
     * 
     * @param shouldUse true if ready to use.
     */
    public boolean shouldUse(HardwareObjects hardware) {
        this.log_active_sensor = (hardware.IMU.getState() == PigeonIMU.PigeonState.Ready);
        return (hardware.IMU.getState() == PigeonIMU.PigeonState.Ready);
    }

    /**
     * Method used to increase angular variance over time
     */
    void updateHeadingVar() {
        this.ang_var = this.ang_var + Constants.DELTA_VAR * Constants.MAIN_DT;
        if (this.ang_var > Constants.MAX_HEADING_VAR) {
            this.ang_var = Constants.MAX_HEADING_VAR;
        }
    }

    /**
     * Resets the values for IMU, also zeroes the accelerometer. Should only be done
     * while there is no motion.
     * 
     * @param hardware Robot hardware objects.
     */
    public void reset(double angle, HardwareObjects hardware) {
        short[] xyz_acc = new short[3];
        hardware.IMU.getBiasedAccelerometer(xyz_acc);
        double x_acc = (double) xyz_acc[0] * -9.81 / 16384;
        double y_acc = (double) xyz_acc[1] * -9.81 / 16384;
        double z_acc = (double) xyz_acc[2] * -9.81 / 16384;

        this.x_acc_zero = x_acc;
        this.xyz_acc_zero[0] = x_acc;
        this.xyz_acc_zero[1] = y_acc;
        this.xyz_acc_zero[2] = z_acc;

        this.yz_mag_zero = SimpleMat.mag(this.xyz_acc_zero) - 9.8;

        hardware.IMU.setFusedHeading(angle);
    }

    /**
     * Projects acceleration vector in the direction of the heading to make more
     * consistent values.
     * 
     * @param state  main robot state.
     * @param xy_acc x and y acceleration values of the robot.
     * 
     * @return acc projected acceleration values.
     */
    double[] projectAcc(MainState state, double[] xy_acc) {
        double xy_mag = SimpleMat.mag(xy_acc);
        double[] acc = { 0, 0 };
        if (xy_mag < this.yz_mag_zero) {
            return acc;
        }
        double[] h_unit = SimpleMat.projectHeading(state.getHeadingVal(), 1);
        double[] projected_acc = SimpleMat.scaleVec(h_unit, SimpleMat.dot(h_unit, xy_acc));
        acc[0] = 0.5 * projected_acc[0] + 0.5 * xy_acc[0];
        acc[1] = 0.5 * projected_acc[1] + 0.5 * xy_acc[1];
        // Forward is -x, back is x 90 deg clockwise
        if (SimpleMat.mag(acc) < 0.2) {
            acc[0] = 0;
            acc[1] = 0;
        }
        return acc;
    }

    /**
     * Get IMU values and project acc, kalman update heading, ang vel, acc
     * 
     * @param state    main robot state.
     * @param hardware robot hardware object.
     */
    public void processValue(MainState state, HardwareObjects hardware) {
        double[] xyz_dps = new double[3];
        short[] xyz_acc = new short[3];
        double[] ypr_deg = new double[3];
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

        hardware.IMU.getRawGyro(xyz_dps);
        hardware.IMU.getFusedHeading(fusionStatus);

        hardware.IMU.getBiasedAccelerometer(xyz_acc);

        hardware.IMU.getYawPitchRoll(ypr_deg);

        SmartDashboard.putNumber("pidgeon/x dps", xyz_dps[0]);
        SmartDashboard.putNumber("pidgeon/y dps", xyz_dps[1]);
        SmartDashboard.putNumber("pidgeon/z dps", xyz_dps[2]);

        SmartDashboard.putNumber("pidgeon/x acc", xyz_acc[0]);
        SmartDashboard.putNumber("pidgeon/y acc", xyz_acc[1]);
        SmartDashboard.putNumber("pidgeon/z acc", xyz_acc[2]);

        SmartDashboard.putNumber("pidgeon/y deg", ypr_deg[0]);
        SmartDashboard.putNumber("pidgeon/p deg", ypr_deg[1]);
        SmartDashboard.putNumber("pidgeon/r deg", ypr_deg[2]);
        // 16384 = 1g

        double r_pitch = ypr_deg[2] * 2 * Math.PI / 360;

        double x_acc = (double) xyz_acc[1] * 9.81 / 16384;
        x_acc = x_acc - x_acc_zero;

        double yt_acc = (double) xyz_acc[0] * -9.81 / 16384;
        double zt_acc = (double) xyz_acc[2] * -9.81 / 16384;
        double y_acc = yt_acc * Math.cos(r_pitch) + zt_acc * Math.sin(r_pitch);

        double[] xy_acc = { y_acc, x_acc * -1 };
        xy_acc = SimpleMat.rot2d(xy_acc, state.getHeadingVal() - Constants.PIDGEON_OFFSET);
        double[] global_acc = xy_acc;

        this.log_acc[0] = x_acc;
        this.log_acc[1] = y_acc;
        this.log_pitch = r_pitch;

        double heading = fusionStatus.heading;
        heading = heading * 2 * Math.PI / 360;
        heading = SimpleMat.angleRectifier(heading);

        this.log_fused_heading = heading;

        double thetas = xyz_dps[2] * -1 * 2 * Math.PI / 360;

        this.log_yaw_vel = thetas;

        double[] kheading = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(), heading, ang_var);
        state.setHeading(kheading[0], kheading[1]);

        double[] kangvel = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(), thetas,
                ang_var / Constants.MAIN_DT);

        state.setAngVel(kangvel[0], kangvel[1]);

        double[] kxacc = state.kalmanUpdate(state.getAccVal()[0], state.getAccVar(), global_acc[0],
                Constants.IMU_ACC_VAR);
        double[] kyacc = state.kalmanUpdate(state.getAccVal()[1], state.getAccVar(), global_acc[1],
                Constants.IMU_ACC_VAR);

        double[] new_acc = { kxacc[0], kyacc[1] };
        state.setAcc(new_acc, kxacc[1]);

        updateHeadingVar();
    }
}