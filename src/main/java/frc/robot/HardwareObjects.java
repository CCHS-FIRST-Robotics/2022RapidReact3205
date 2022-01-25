package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.kauailabs.navx.frc.*;

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
    public AHRS NAVX;

    /**
     * Constructor for HardwareObjects. Handles creation and initialization.
     */
    public HardwareObjects() {
        this.FLD_MOTOR = new WPI_TalonSRX(Constants.FL_TALON_PORT);
        this.FRD_MOTOR = new WPI_TalonSRX(Constants.FR_TALON_PORT);
        this.BLD_MOTOR = new WPI_TalonSRX(Constants.BL_TALON_PORT);
        this.BRD_MOTOR = new WPI_TalonSRX(Constants.BR_TALON_PORT);
        this.NAVX = new AHRS();
        //IMU = new PigeonIMU(RIGHT_MOTOR2);

        this.FLD_MOTOR.configFactoryDefault();
        this.FRD_MOTOR.configFactoryDefault();
        this.BLD_MOTOR.configFactoryDefault();
        this.BRD_MOTOR.configFactoryDefault();
        //IMU.configFactoryDefault();
        //IMU.setFusedHeading(0.0, Constants.TIMEOUT_MS);
        this.NAVX.reset();
        this.NAVX.calibrate();
        while (this.NAVX.isCalibrating()){

        }
        this.NAVX.zeroYaw();
        this.NAVX.setAngleAdjustment(0);

    }
}