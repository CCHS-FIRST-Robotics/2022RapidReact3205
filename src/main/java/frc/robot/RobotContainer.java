// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Calendar;
import frc.robot.commands.CommandHandler;
import frc.robot.ai.AI;
import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.commands.UpdateState;
import frc.robot.sensors.ADGyroSensor;
import frc.robot.sensors.DriveEncoderSensor;
import frc.robot.sensors.NAVXSensor;
import frc.robot.sensors.NAVXAccumSensor;
import frc.robot.sensors.RoboRioAccSensor;
import frc.robot.sensors.LidarSensor;
import frc.robot.sensors.StorageLidar;
import frc.robot.sensors.ShooterEncoder;
import frc.robot.sensors.InternalIRBeamSensor;
import frc.robot.sensors.IMUSensor;
import frc.robot.sensors.Limelight;
import frc.robot.HardwareObjects;
import frc.robot.network.*;
import frc.robot.map.Map;

import static frc.robot.Constants.*;

/**
 * RobotContainer class, contains every class of the robot and is constantly
 * running.
 * 
 * @author Ludwig Tay
 */
public class RobotContainer {
  public double controller_start_time = System.currentTimeMillis() / 1000;
  public Map map = new Map();
  public MainState main_state = new MainState();
  // public SomeSensor some_sensor = new SomeSensor();
  public Calendar main_timer = Calendar.getInstance();
  double SYNC_TIME = 0;
  public DriveEncoderSensor drive_encoder_sensor = new DriveEncoderSensor(SYNC_TIME);
  public NAVXSensor navx_sensor = new NAVXSensor(SYNC_TIME);
  public AI ai = new AI();
  public CommandHandler command_handler = new CommandHandler();
  public Command main_command = new Command(Constants.DEFAULT_CMD);
  public HardwareObjects hardware;
  public Network network;
  public NAVXAccumSensor navx_accum;
  public StorageLidar s_lidar;

  public ADGyroSensor ad_gyro;
  public RoboRioAccSensor rr_acc;
  public LidarSensor lidar;
  public ShooterEncoder shooter_e;
  public InternalIRBeamSensor beam;
  public IMUSensor imu;
  public Limelight lime;
  double log;

  /**
   * RobotContainer Constructor.
   */
  public RobotContainer() {
    this.map = new Map();
    this.ai = new AI();
    this.command_handler = new CommandHandler();
    this.main_command = new Command(Constants.DEFAULT_CMD);
    this.SYNC_TIME = (double) main_timer.getTimeInMillis() / 1000;
    this.drive_encoder_sensor = new DriveEncoderSensor(SYNC_TIME);
    this.navx_sensor = new NAVXSensor(SYNC_TIME);
    this.hardware = new HardwareObjects();
    this.network = new Network();
    this.navx_accum = new NAVXAccumSensor(this.main_state, this.hardware, SYNC_TIME);
    this.ad_gyro = new ADGyroSensor(SYNC_TIME);
    this.rr_acc = new RoboRioAccSensor(SYNC_TIME);
    this.lidar = new LidarSensor(SYNC_TIME);
    this.s_lidar = new StorageLidar(SYNC_TIME);
    this.shooter_e = new ShooterEncoder(SYNC_TIME);
    this.beam = new InternalIRBeamSensor(SYNC_TIME);
    this.imu = new IMUSensor(SYNC_TIME);
    this.lime = new Limelight(SYNC_TIME);
    reset();
  }

  public void init() {

  }

  /**
   * Reset values in robot container.
   */
  public void reset() {
    this.navx_sensor.reset(0, this.hardware);
    this.main_command = new Command(Constants.DEFAULT_CMD);
    this.main_state = this.map.initialize(this.hardware);
    this.network.init(SYNC_TIME);
    this.navx_accum.reset(this.main_state, this.hardware);
    this.ad_gyro.reset(this.hardware);
    this.rr_acc.reset(this.hardware);
    this.lidar.reset();
    this.s_lidar.reset(this.hardware);
    this.hardware.resetMotors();
  }

  /**
   * mainLoop that executes state update, predict, ai and command scheduling.
   */
  public void mainLoop() {
    this.hardware.printMotorHealth();
    SmartDashboard.putNumberArray("Start Pos", Constants.START_POS);
    SmartDashboard.putNumberArray("Start", Constants.START);
    Constants.START_POS = new double[] { Constants.START[0], Constants.START[1] };
    this.main_command = this.ai.getCommand(this.hardware, this.main_state, this.map, this.controller_start_time,
        this.network);

    UpdateState.updateState(this.main_state, this.main_command);
    this.command_handler.scheduleCommands(this.main_command, this.hardware);

    if (this.drive_encoder_sensor.shouldUse()) {
      this.drive_encoder_sensor.processValue(this.main_state, this.hardware);
    }
    if (this.navx_sensor.shouldUse(this.hardware)) {
      this.navx_sensor.processValue(this.main_state, this.hardware);
    }
    if (this.shooter_e.shouldUse()) {
      this.shooter_e.processValue(this.main_state, this.hardware);
    }
    if (this.ad_gyro.shouldUse(this.hardware)) {
      this.ad_gyro.processValue(this.main_state, this.hardware);
    }
    if (this.rr_acc.shouldUse()) {
      this.rr_acc.processValue(this.main_state, this.hardware);
    }
    if (this.beam.shouldUse()) {
      this.beam.processValue(this.main_state, this.hardware);
    }
    if (this.imu.shouldUse(hardware)) {
      this.imu.processValue(this.main_state, this.hardware);
    }
    boolean use_lime = false;
    if (this.lime.shouldUse(this.main_state, this.network)) {
      this.lime.processValue(this.main_state, this.network);
      use_lime = true;
    }
    SmartDashboard.putBoolean("Limelight/use_lime", use_lime);
    SmartDashboard.putNumber("variance/pos", main_state.getPosVar());
    SmartDashboard.putNumber("variance/vel", main_state.getVelVar());
    SmartDashboard.putNumber("variance/acc", main_state.getAccVar());
    this.map.getBalls(this.network);

    this.main_state.predict(Constants.MAIN_DT);
    // SmartDashboard.putNumberArray("PosArr",this.main_state.getPosVal());

    this.network.writeNTable(this.main_state);

    this.log = this.network.stereo_net.getHeadingVal();

  }

  /**
   * Sets finite state in AI to controller.
   */
  public void setControllerState() {
    this.ai.setControllerState();
  }

  /**
   * Sets finite state in AI to auton.
   */
  public void setAutonomousState() {
    this.ai.setAutonomousState(this.hardware, this.main_state, this.map, this.network);
  }

}