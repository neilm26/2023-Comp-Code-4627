// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Drivetrain extends SubsystemBase {
  private TalonFX m_frontLeft = new TalonFX(Constants.MOTORS.LEFT_MOTOR_1.value);
  private TalonFX m_rearLeft = new TalonFX(Constants.MOTORS.LEFT_MOTOR_2.value);

  private TalonFX m_frontRight = new TalonFX(Constants.MOTORS.RIGHT_MOTOR_1.value);
  private TalonFX m_rearRight = new TalonFX(Constants.MOTORS.RIGHT_MOTOR_2.value);
  

  private final PID m_PID;
  

  private final double distancePerTick = (Constants.WHEEL_CIRCUMFERENCE/(Constants.ENCODER_PULSES_PER_ROTATION*1.4*Constants.ENCODER_GEAR_RATIO));
  // we really need to find what's causing the error here so we can remove *1.4
  
  private boolean visionRunning = false;

  private TalonFX ConfigDriveAttributes(TalonFX motor) {
    motor.configNeutralDeadband(0.001); //dead zone where nothing happens; similar to deadband on controller

    motor.configNominalOutputForward(0.0, Constants.TIMEOUT_MS);
    motor.configNominalOutputReverse(0.0, Constants.TIMEOUT_MS);
    motor.configPeakOutputForward(1.0, Constants.TIMEOUT_MS);
    motor.configPeakOutputReverse(-1.0, Constants.TIMEOUT_MS);  

    return motor;
  }

  /** Creates a new Drive. */
  public Drivetrain(PID pid) {
    // m_drive.setMaxOutput(distancePerTick);
    // resetEncoders();
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_frontLeft.setNeutralMode(NeutralMode.Brake);                                                                      
    m_rearLeft.setNeutralMode(NeutralMode.Brake);                                                                     
    m_rearRight.setNeutralMode(NeutralMode.Brake);
    m_PID = pid;
                                                                        
    m_frontLeft = ConfigDriveAttributes(m_frontLeft);
    m_frontRight = ConfigDriveAttributes(m_frontRight);
    m_rearRight = ConfigDriveAttributes(m_rearRight);
    m_rearLeft = ConfigDriveAttributes(m_rearLeft);

    // m_frontLeft.setInverted(false);
    // m_frontLeft.setInverted(true); // Left side mounted backwards
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
    SmartDashboard.putNumber("Applied Velocity: ", m_frontRight.getSelectedSensorVelocity(0));
    double left = Utilities.scale(leftVel, scaleFactor);
    //left is joystick input multiplied by a scaling factor to limit max_vel
    //drive compensation is tunable
    //peak drive rpm is the max rpm of a falcon 500 taking into account robot weight
    m_frontLeft.set(TalonFXControlMode.Velocity, -left * Constants.PEAK_DRIVE_RPM * Constants.TICKS_PER_DRIVE_TRAIN_REVOLUTION / Constants.DRIVE_COMPENSATION);
  }

  public double getSensorVel(TalonFX talon) {
    return talon.getSelectedSensorVelocity();
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
