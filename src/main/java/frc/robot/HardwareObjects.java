package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

/**
 * Class that contains and initializes for all hardware objects
 * 
 * @author Ludwig Tay
 */
public class HardwareObjects {
    public TalonSRX FLD_MOTOR;
    public TalonSRX FRD_MOTOR;
    public TalonSRX BLD_MOTOR;
    public TalonSRX BRD_MOTOR;
    public PigeonIMU IMU;

    /**
     * Constructor for HardwareObjects. Handles creation and initialization.
     */
    public HardwareObjects() {
        this.FLD_MOTOR = new WPI_TalonSRX(Constants.L_TALON_PORT1);
        this.FRD_MOTOR = new WPI_TalonSRX(Constants.L_VICTOR_PORT2);
        this.BLD_MOTOR = new WPI_TalonSRX(Constants.R_TALON_PORT1);
        this.BRD_MOTOR = new WPI_TalonSRX(Constants.R_TALON_PORT2);
        IMU = new PigeonIMU(RIGHT_MOTOR2);

        this.FLD_MOTOR.configFactoryDefault();
        this.FRD_MOTOR.configFactoryDefault();
        this.BLD_MOTOR.configFactoryDefault();
        this.BRD_MOTOR.configFactoryDefault();
        IMU.configFactoryDefault();
        IMU.setFusedHeading(0.0, Constants.TIMEOUT_MS);
    }
}