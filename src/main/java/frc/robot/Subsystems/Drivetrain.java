// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Drivetrain extends SubsystemBase {
  private final TalonFX m_frontLeft = new TalonFX(Constants.MOTORS.LEFT_MOTOR_1.value);
  private final TalonFX m_rearLeft = new TalonFX(Constants.MOTORS.LEFT_MOTOR_2.value);

  private final TalonFX m_frontRight = new TalonFX(Constants.MOTORS.RIGHT_MOTOR_1.value);
  private final TalonFX m_rearRight = new TalonFX(Constants.MOTORS.RIGHT_MOTOR_2.value);
  

  private final PID m_PID;
  

  private final double distancePerTick = (Constants.WHEEL_CIRCUMFERENCE/(Constants.ENCODER_PULSES_PER_ROTATION*1.4*Constants.ENCODER_GEAR_RATIO));
  // we really need to find what's causing the error here so we can remove *1.4
  
  private boolean visionRunning = false;

  /** Creates a new Drive. */
  public Drivetrain(PID pid) {
    // m_drive.setMaxOutput(distancePerTick);
    // resetEncoders();
    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);
    m_PID = pid;    

    // m_frontLeft.setInverted(false);
    // m_frontLeft.setInverted(true); // Left side mounted backwards
  }

  public void setMotors(double right, double left) {
    right = Utilities.scale(right, Constants.DRIVE_MAX_SPEED);
    m_frontRight.set(ControlMode.PercentOutput, right);

    left = Utilities.scale(left, Constants.DRIVE_MAX_SPEED);
    m_frontLeft.set(ControlMode.PercentOutput, -left);
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
