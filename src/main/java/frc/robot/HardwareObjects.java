package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.Counter;
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

    public VictorSPX INTAKE_MOTOR;
    public TalonSRX STORAGE_1_MOTOR;

    public TalonSRX STORAGE_2_MOTOR;
    public TalonSRX SHOOTER_1_MOTOR;
    public TalonSRX SHOOTER_2_MOTOR;

    public PigeonIMU IMU;
    public AHRS NAVX;
    public ADXRS450_Gyro AD_GYRO;
    public Accelerometer RR_ACC;
    public Counter S_LIDAR;

    /**
     * Constructor for HardwareObjects. Handles creation and initialization.
     */
    public HardwareObjects() {
        this.FLD_MOTOR = new WPI_TalonSRX(Constants.FL_TALON_PORT);
        this.FRD_MOTOR = new WPI_TalonSRX(Constants.FR_TALON_PORT);
        this.BLD_MOTOR = new WPI_TalonSRX(Constants.BL_TALON_PORT);
        this.BRD_MOTOR = new WPI_TalonSRX(Constants.BR_TALON_PORT);

        this.INTAKE_MOTOR = new WPI_VictorSPX(Constants.INTAKE_TALON_PORT);
        this.STORAGE_1_MOTOR = new WPI_TalonSRX(Constants.STORAGE_1_TALON_PORT);

        this.STORAGE_2_MOTOR = new WPI_TalonSRX(Constants.STORAGE_2_TALON_PORT);
        this.SHOOTER_1_MOTOR = new WPI_TalonSRX(Constants.SHOOTER_1_TALON_PORT);
        this.SHOOTER_2_MOTOR = new WPI_TalonSRX(Constants.SHOOTER_2_TALON_PORT);

        this.S_LIDAR = new Counter(Constants.S_LIDAR);

        this.NAVX = new AHRS();
        // IMU = new PigeonIMU(RIGHT_MOTOR2);

        this.FLD_MOTOR.configFactoryDefault();
        this.FRD_MOTOR.configFactoryDefault();
        this.BLD_MOTOR.configFactoryDefault();
        this.BRD_MOTOR.configFactoryDefault();

        this.INTAKE_MOTOR.configFactoryDefault();
        this.STORAGE_1_MOTOR.configFactoryDefault();
        // IMU.configFactoryDefault();
        // IMU.setFusedHeading(0.0, Constants.TIMEOUT_MS);

        this.S_LIDAR.setMaxPeriod(1.00);
        this.S_LIDAR.setSemiPeriodMode(true);
        this.S_LIDAR.reset();

        this.NAVX.reset();
        this.NAVX.calibrate();
        while (this.NAVX.isCalibrating()) {

        }
        this.NAVX.zeroYaw();
        this.NAVX.setAngleAdjustment(0);

        this.AD_GYRO = new ADXRS450_Gyro();
        this.AD_GYRO.calibrate();

        this.RR_ACC = new BuiltInAccelerometer();

    }
}