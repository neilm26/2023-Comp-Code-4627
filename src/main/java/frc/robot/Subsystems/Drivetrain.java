// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Drivetrain extends SubsystemBase {
  private TalonFX m_frontLeft = new TalonFX(Constants.MOTORS.LEFT_MOTOR_1.value);
  private TalonFX m_rearLeft = new TalonFX(Constants.MOTORS.LEFT_MOTOR_2.value);

  private TalonFX m_frontRight = new TalonFX(Constants.MOTORS.RIGHT_MOTOR_1.value);
  private TalonFX m_rearRight = new TalonFX(Constants.MOTORS.RIGHT_MOTOR_2.value);
  private TimeOfFlight m_tof = new TimeOfFlight(Constants.TIME_OF_FLIGHT_ID);  

  private final PID m_PID;
  private final NetworktablesUpdated m_table;
  private double startDist;

  private TalonFX ConfigDriveAttributes(TalonFX motor) {

    motor.configFactoryDefault();

    
    motor.configNeutralDeadband(0.001); //dead zone where nothing happens; similar to deadband on controller
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,Constants.PID_IDX, Constants.TIMEOUT_MS);
    motor.configNominalOutputForward(0.0, Constants.TIMEOUT_MS);
    motor.configNominalOutputReverse(0.0, Constants.TIMEOUT_MS);
    motor.configPeakOutputForward(1.0, Constants.TIMEOUT_MS);
    motor.configPeakOutputReverse(-1.0, Constants.TIMEOUT_MS);  
    

    motor.config_kP(Constants.PID_IDX, 0.05);
    return motor;
  }

  /** Creates a new Drive. */
  public Drivetrain(PID pid, NetworktablesUpdated m_table) {

    m_rearLeft.set(ControlMode.Follower, m_frontLeft.getDeviceID());
    m_rearRight.set(ControlMode.Follower, m_frontRight.getDeviceID());


    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_frontLeft.setNeutralMode(NeutralMode.Brake);                                                                      
    m_rearLeft.setNeutralMode(NeutralMode.Brake);                                                                     
    m_rearRight.setNeutralMode(NeutralMode.Brake);
    m_PID = pid;
    this.m_table = m_table;
                                                                        
    m_frontLeft = ConfigDriveAttributes(m_frontLeft);
    m_frontRight = ConfigDriveAttributes(m_frontRight);
    m_rearRight = ConfigDriveAttributes(m_rearRight);
    m_rearLeft = ConfigDriveAttributes(m_rearLeft);

    // m_frontLeft.setInverted(false);
    // m_rearLeft.setInverted(true); // Left side mounted backwards
    // m_frontRight.setInverted(true);
  }

  public void setMotors(double right, double left, double max) {
    right = Utilities.scale(right, max);
    m_frontRight.set(ControlMode.PercentOutput, right);

    left = Utilities.scale(left, max);
    m_frontLeft.set(ControlMode.PercentOutput, -left);
  }

  public void setMotorsVelocity(double rightVel, double leftVel, double scaleFactor) {
    double right = Utilities.scale(rightVel, scaleFactor);
    m_frontRight.set(TalonFXControlMode.Velocity, right * Constants.PEAK_DRIVE_RPM * Constants.TICKS_PER_DRIVE_TRAIN_REVOLUTION / Constants.DRIVE_COMPENSATION);   

    double left = Utilities.scale(leftVel, scaleFactor);
    //left is joystick input multiplied by a scaling factor to limit max_vel
    //drive compensation is tunable
    //peak drive rpm is the max rpm of a falcon 500 taking into account robot weight
    m_frontLeft.set(TalonFXControlMode.Velocity, -left * Constants.PEAK_DRIVE_RPM * Constants.TICKS_PER_DRIVE_TRAIN_REVOLUTION / Constants.DRIVE_COMPENSATION);
  }

  public double getToFDistance() {
    return m_tof.getRange();
  }

  public double[] getSensorValues() {
    double[] tmp = new double[4];

    tmp[0] = -m_frontLeft.getSelectedSensorPosition()-m_rearLeft.getSelectedSensorPosition();
    tmp[1] = m_frontRight.getSelectedSensorPosition()+m_rearRight.getSelectedSensorPosition();
    return tmp;
  }

  public void encoderForward(double distance) {
    m_frontLeft.setSelectedSensorPosition(distance, Constants.PID_IDX, Constants.TIMEOUT_MS);
    m_frontRight.setSelectedSensorPosition(distance, Constants.PID_IDX, Constants.TIMEOUT_MS);
  }

  public HashMap<String, Double> shfVals() {
    
    double m_balance_kP = m_table.balance_kp_entry.getDouble(0);
    double m_balance_kI = m_table.balance_ki_entry.getDouble(0);
    double m_balance_kD = m_table.balance_kd_entry.getDouble(0);

    double m_ramp_kP = m_table.ramp_kp_entry.getDouble(0);
    double m_ramp_kI = m_table.ramp_ki_entry.getDouble(0);
    double m_ramp_kD = m_table.ramp_kd_entry.getDouble(0);

    HashMap<String, Double> dict = new HashMap<String, Double>();

    dict.put("balancekP", m_balance_kP);
    dict.put("balancekI", m_balance_kI);
    dict.put("balancekD", m_balance_kD);

    dict.put("rampkP", m_ramp_kP);
    dict.put("rampkI", m_ramp_kI);
    dict.put("rampkD", m_ramp_kD);

    return dict;
  }

  public double getConvertedToMeters(double[] sensorInput) {
    double averagedDist = 0;
    for (double inp:sensorInput) {
      double motorRot = inp / Constants.TICKS_PER_DRIVE_TRAIN_REVOLUTION;
      double wheelRot = motorRot / Constants.ENCODER_GEAR_RATIO;
      averagedDist += wheelRot * (2 * Math.PI * Units.inchesToMeters(Constants.WHEEL_DIAMETER / 2));
    }
    return averagedDist / sensorInput.length;
  }

  public void setStartDist(double override) {
      startDist = override;
  }

  public double getStartDist() {
    return startDist;
  }

  public void resetEncoders() {
    m_frontLeft.setSelectedSensorPosition(0);
    m_frontRight.setSelectedSensorPosition(0);
    m_rearLeft.setSelectedSensorPosition(0);
    m_rearRight.setSelectedSensorPosition(0);
  }

  public double getBuiltInEncoder(TalonFX talon) {
    return talon.getSelectedSensorPosition();
  }

  public void setToFMode(RangingMode inp) {
    m_tof.setRangingMode(inp,Constants.TIMEOUT_MS);
  }
  
  public void voltageCompensation(double limitVoltage, boolean activate) {
    m_frontLeft.configVoltageCompSaturation(limitVoltage);
    m_frontRight.configVoltageCompSaturation(limitVoltage);
    m_rearLeft.configVoltageCompSaturation(limitVoltage);
    m_rearRight.configVoltageCompSaturation(limitVoltage);
    
    m_frontLeft.enableVoltageCompensation(activate);
    m_frontRight.enableVoltageCompensation(activate);
    m_rearLeft.enableVoltageCompensation(activate);
    m_rearRight.enableVoltageCompensation(activate);

  }

  @Override
  public void periodic() {
  }
  public void setvisionRunning(boolean booleanb) {

  }

  public PID getController() {
    return m_PID;
  }
}
