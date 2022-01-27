// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ==================
    // HARDWARE PORTS
    // ==================]
    public static final int FL_TALON_PORT = 2;
    public static final int FR_TALON_PORT = 1;
    public static final int BL_TALON_PORT = 3;
    public static final int BR_TALON_PORT = 4;

    public static final int IMU_PORT = 6;

    public static final int XBOX_PORT = 0;

    // ==================
    // STATE INITIALIZATION
    // ==================
    public static final double[] INIT_POS = { 0, 0 };
    public static final double[] INIT_VEL = { 0, 0 };
    public static final double[] INIT_ACC = { 0, 0 };

    public static final double INIT_HEADING = 0;
    public static final double INIT_ANG_VEL = 0;
    public static final double INIT_ANG_ACC = 0;

    public static final double INIT_L_WHL_VEL = 0;
    public static final double INIT_R_WHL_VEL = 0;

    public static final double INIT_L_WHL_TRAC = 1;
    public static final double INIT_R_WHL_TRAC = 1;

    public static final double INIT_FRICTION = 0.2;

    public static final double INIT_MIX = 0.5;

    public static final double INIT_DW = 0.4;

    public static final double INIT_VARIANCE = 0.001;

    public static final double INIT_WHL_RPM = 0;
    // ==================
    // VARIANCE LINEAR APPROX
    // ==================
    public static final double ANG_VEL_VARIANCE = 1.5 * 1.5;
    public static final double ACC_VARIANCE = 2 * 2;

    // ==================
    // ENCODER VARIANCE
    // ==================
    public static final double VAR_RAD_VAR = 0.1 * 0.1 / 20;

    // ==================
    // IMU VARIANCE
    // ==================
    public static final double BASE_HEADING_VAR = 0.1 * 0.1;
    public static final double DELTA_VAR = 0.0001;
    public static final double MAX_HEADING_VAR = 0.2 * 0.2;

    public static final double IMU_ACC_VAR = 4;
    // ==================
    // TIME CONSTANTS
    // ==================
    public static final double MAIN_DT = 0.04;

    // ==================
    // MOTOR CONSTANTS
    // ==================
    public static final double GEAR_RATIO = 10.71;

    public static final double MOTOR_MAX_POWER = 337; // OUTPUT IN WATTS

    public static final double MOTOR_MAX_TORQUE = 2.41 * GEAR_RATIO;

    public static final double MOTOR_MAX_RPM = 5330 / GEAR_RATIO;

    public static final double MOTOR_PROP_VAR = 0.1;

    public static final int TIMEOUT_MS = 50;

    // ==================
    // CERTAIN PHYSICAL CONSTANTS
    // ==================
    public static final double ROBOT_WIDTH = 0.2667;
    public static final double ROBOT_LENGTH = 0.5207;
    public static final double WHEEL_RADIUS = 0.0726;
    public static final double WHEEL_MASS = 0.5669905;
    public static final double WHEEL_MOI = 0.5 * WHEEL_RADIUS * WHEEL_RADIUS * WHEEL_MASS;
    public static final double ROBOT_MASS = 12;
    public static final double MOI = 0.5 * ROBOT_LENGTH * ROBOT_LENGTH * ROBOT_MASS;

    public static final double GRAV_ACC = -9.81;

    // ==================
    // PATHING CONSTANTS
    // ==================
    public static final double NO_ORIENT_DIST = 0.5;

    public static final double TIMER_START = 1;
    public static final double TIMER_LEEWAY = 8;

    public static final double ACCEPTABLE_DIST_ERROR = 0.03;

    public static final double ACCEPTABLE_DIST_SOFT = 0.4;

    public static final double ACCEPTABLE_ANGLE_ERROR = 0.05;

    public static final double EXIT_THETA = 1.2217;

    public static final double TURN_LEEWAY = 6;

    public static final double TURN_TIME_FAC = 0.4;
    public static final double TURN_TIME_MAX = 2;
    public static final double MIN_R_FAC = 0.8;

    // Standard PID Tunings
    public static final double[] DRIVE_WHL_TUNING = { 0.1, 0.1, 0.1 };

    public static final double[] TURN_TUNING = { 0.2, 0.1, 0.05 };

    public static final double[] FORWARD_TUNING = { 0.3, 0.05, 0.4 };

    // Controller Tunings
    public static final double C_BASE_GAIN = 0.02;
    public static final double[] C_BASE_PID = { 0.01, 0.00001, 0.0 };
    public static final double INTERGRAL_HALFLIFE_T = 8;

}
