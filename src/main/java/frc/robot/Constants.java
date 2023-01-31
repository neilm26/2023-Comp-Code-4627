// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // ----------- REMOTE VALUES ------------ //
  public static final int DRIVER_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;

  public static final int BUTTON_A = 1;
  public static final int BUTTON_B = 2;
  public static final int BUTTON_X = 3;
  public static final int BUTTON_Y = 4;
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int BACK_BUTTON = 7;
  public static final int START_BUTTON = 8;
  public static final int LEFT_STICK_BUTTON = 9;
  public static final int RIGHT_STICK_BUTTON = 10;

  public static final int DPAD_UP = 0;
  public static final int DPAD_UPRIGHT = 45;
  public static final int DPAD_RIGHT = 90;
  public static final int DPAD_DOWN = 180;
  public static final int DPAD_LEFT = 270;
  public static final int DPAD_UPLEFT = 315;

  public static final int LEFT_STICK_X = 0;
  public static final int LEFT_STICK_Y = 1;
  public static final int RIGHT_STICK_Y = 5;
  public static final int RIGHT_STICK_X = 4;

  public static final int LEFT_TRIGGER = 2;
  public static final int RIGHT_TRIGGER = 3;


  public static enum MOTORS {
    LEFT_MOTOR_1(2), // Falcon500 (TalonFX) 0 (2) for snoopy
    LEFT_MOTOR_2(3), // Falcon500 (TalonFX) 1 (3) for snoopy
    RIGHT_MOTOR_1(1), // Falcon500 (TalonFX) 2 (1) for snoopy
    RIGHT_MOTOR_2(5), // Falcon500 (TalonFX) 3 (5) for snoopy
    SHOOTER_MOTOR(4), // Falcon500 (TalonFX) 4
    //TURRET_MOTOR(5), // Mini Neo 550 (Spark) 5
    INTAKE_EXTENSION_MOTOR(6), // Mini Neo 550 (Spark) 6
    ELEVATOR_MOTOR1(7), // // Falcon500 (TalonFX) 7
    ELEVATOR_MOTOR2(8), // Falcon500 (TalonFX) 8xx
    STORAGE_MOTOR(9), // Neo (Spark) 9
    CLIMBER_BOX_MOTOR2(10), // Falcon500 (TalonFX) 11
    CLIMBER_BOX_MOTOR1(11), // Falcon500 (TalonFX) 11
    INTAKE_MOTOR(12); // Mini Neo 550 (Spark) 12
    
    public final int value;
    
    private MOTORS(int value) {
      this.value = value;
    }
  }
  

  
  // ----------- SUBSYSTEM CONSTANTS ------------ //
  
  //Timeout
  public static final int TIMEOUT_MS = 30;

  // CLIMBER

  // Climb
  public static final double CLIMBER_P = 0.25;
  public static final double CLIMBER_I = 0;
  public static final double CLIMBER_D = 0;
  
  public static final double CLIMBER_SETPOINT_TOLERANCE = 3;
  public static final double MAX_CLIMB_CURRENT = 10;
  public static final double CLIMBER_P_LOW = 0.1;
  
  public static final double MIN_CIAB_HEIGHT = -27;
  public static final double MAX_CIAB_HEIGHT = 202;

  // Elevator
  public static final double TILT_P = 0.02;
  public static final double TILT_I = 0.000003;
  public static final double TILT_D = 0.000006;

  public static final double TILT_MAX_SPEED = 0.5;
  public static final double TILT_SETPOINT = 50;//prev 65
  public static final double TILT_SETPOINT_GRAB = 50;
  public static final double TILT_TRAVERSE = 40;
  public static final double TILT_TRAVERSE_GRAB = 56;
  
  
  // DRIVETRAIN
  public static double DRIVE_MAX_SPEED = 0.3;
  public static final double DRIVE_RAMP_RATE = 1; // ramp rate describes time for the motor to reach max speed
  public static final double SPEED_MULTIPLIER = 0.75;

  public static final double DRIVE_ANG_ERR_TOL = 0.4; //angle tolerance of +- said value in degrees

  public static final double DRIVE_VEL_PEAK = 12; //max voltage
  public static final int PID_IDX = 0; //pid_controller identification
  public static final double TICKS_PER_DRIVE_TRAIN_REVOLUTION = 2048;
  public static final double PEAK_DRIVE_RPM = 2000;
  public static final double DRIVE_COMPENSATION = 600;

  public static final double RAMP_KP = 0.3;
  public static final double RAMP_KI = 0.07;
  public static final double RAMP_KD = 0;

  //Inclination
  public static final double MAX_INCLINATION_ANGLE = 25;
  public static final double BALANCE_LEN = 1.0;
  ///////////////////////////////
  
  // Encoder values
  public static final double WHEEL_DIAMETER = 6;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER - Math.PI;
  public static final double ENCODER_PULSES_PER_ROTATION = 2048;
  public static final double ENCODER_GEAR_RATIO = 8.7; //laughably inaccurate
  
  
  // SHOOTER
  // public static final double MAX_SHOOTER_RPM = 6380;
  public static double SHOOTER_KP = 0.3;
  public static double SHOOTER_KI = 0.00026;
  public static double SHOOTER_KD = 0.000012;

  public static double SHOOTER_KS = 0.63503;
  public static double SHOOTER_KV = 0.22891;
  public static double SHOOTER_KA = 0.017649;

  public static final double BACKSPIN_CORRECTION_CONSTANT = 0.95;
  public static final double SHOOTER_MULTIPLIER = 3; // not yet used
  public static double SHOOTER_MAX_SPEED = 0.5; // not yet used
  
  
  // INTAKE
  public static final double INTAKE_CURRENT_LIMIT = 3.3;
  public static final double INTAKE_EXTENSION_KP = 0.025;//0.01;
  public static final double INTAKE_EXTENSION_KI = 0.00003;//0.00001;
  public static final double INTAKE_EXTENSION_KD = 0;
  public static final double INTAKE_EXTENSION_RATIO = (9.0/1.0) * (30.0/18.0); // 9 to 1 gearbox, 18 tooth gear going along 40 teeth

  
  // TURRET
  public static double TURRET_KP = 0.04;//0.032
  public static double TURRET_KI = 0.00001;//0.00003
  public static double TURRET_KD = 0;//0.0002

  public static double TURRET_KS = 0;
  public static double TURRET_KV = 0;
  
  public static final double MAX_TURRET_ERROR_TOLERANCE = 1.8; //1.8
  public static final double MIN_TURRET_ERROR_TOLERANCE = 0.6; //0.6
  public static final double TURRET_RAMP_RATE = 0.2;
  
  public static final double TURRET_FORWARD_LIMIT = 12.07;
  public static final double TURRET_REVERSE_LIMIT = -12.07;
  public static final double HEIGHT_OF_TURRET = 1.6; // !!! value must be updated !!!
  
  // encoder values
  public static final double TURRET_GEAR_RATIO = 47.497888929;//42.139903796;
  public static final double TURRET_ENCODER_EPR = 4096;
  // *if changing turret motor to NEO, these values shouldn't be needed
  
  public static final double INITIAL_FIRING_ANGLE = 75;
  public static final double HEIGHT_OF_SHOOTER = 0.6; //in meters
  public static final double HEIGHT_OF_STRUCTURE = 2.5146; //in meters
  public static final double SLIMELIGHT_ANGLE = 0; // in the instance that distance from hoop is calculated based on angle of camera
  

    //PIGEON
    public static final double PIGEON_KP = 0.0485; //.051835
    public static final double PIGEON_KI = 0.0016666666666;
    public static final double PIGEON_KD = 0;
  
    public static final double FEEDFORWARD_KV = 0.03; //use sysid
    public static final double FEEDFORWARD_KA = 0.03;
    public static final double FEEDFORWARD_KS = 0.0;
    public static final double FEEDFORWARD_ANG_KV = 0.03;
    public static final double FEEDFORWARD_ANG_KA = 0.03;

    public static final double PIGEON_TOLERANCE = 5;
  
  // ----------- COMMAND CONSTANTS ------------ //
  // AutoDrive
  public static final double MAX_TURN_SPEED = 1;
  public static final double CORRECTION_SPEED = 0.1;
  
  // All linear interpolation pid values should be replaced with ONE accurate set of values when pid is tuned
  public static final double DRIVING_KP_CLOSE = 0.0049;
  public static final double DRIVING_KI_CLOSE = 0.00004;
  public static final double DRIVING_KD_CLOSE = 0.000001;

  public static final double DRIVING_KP_FAR = 0.0031;
  public static final double DRIVING_KI_FAR = 0.0001;
  public static final double DRIVING_KD_FAR = 0.0006;

  // KP = 0.0075
  // KI = 0.00035
  // KD = 0.00001
  
  // correction pid  values shouldn't be any different from normal 
  public static final double CORRECTION_KP = 0.15;
  public static final double CORRECTION_KI = 0.05;
  public static final double CORRECTION_KD = 0.01;


  // AUTO TURN
  // All linear interpolation pid values should be replaced with ONE accurate set of values when pid is tuned
  public static final double TURN_STEERING_KP_CLOSE = 0.002500; 
  public static final double TURN_STEERING_KI_CLOSE = 0.001800;
  public static final double TURN_STEERING_KD_CLOSE = 0.000030;
  
  public static final double TURN_STEERING_KP_FAR = 0.003000;
  public static final double TURN_STEERING_KI_FAR = 0.000400;
  public static final double TURN_STEERING_KD_FAR = 0.000000;
  
  
  // UNUSED VALUES AS OF NOW
  public static final double TURNING_RATE = 0.5;
  public static final int NEO550_CURRENT_LIMIT = 0;
  public static final int CURRENT_LIMIT = 70;
  public static final int CONTINUOUS_CURRENT_LIMIT = 35;
  public static final int CURRENT_LIMIT_DURATION = 50;
  public static final int PIGEON_ID = 6; //(6) for snoopy
  public static final int TIME_OF_FLIGHT_ID = 0;
   
  public static final double RADIUS_OF_FLYWHEEL = 0.08; //in meters
  public static final double TICKS_PER_FLYWHEEL_REVOLUTION = 2048; //ticks
  

  // GENERAL
  public static final double GRAVITY = 9.81; //m per s^2
public static final double INTAKE_EXTENSION_FAILSAFE = 1.4;
public static final double INTAKE_EXTENSION_MIN = 0.5;
public static final Gains VEL_GAINS = new Gains(PIGEON_KP, PIGEON_KI, PIGEON_KD, TICKS_PER_DRIVE_TRAIN_REVOLUTION/20660.0, 300, 1.00);

}
