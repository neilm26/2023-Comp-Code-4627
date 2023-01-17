// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PID extends PIDSubsystem {
  /** Creates a new PID. */

  public PID(double kp, double ki, double kd) {
    super(
        // The PIDController used by the subsystem
        new PIDController(kp, ki, kd));
  }

  public double returnCalc(double curr, double setpoint) {
    // Use the output here
    return getController().calculate(curr, setpoint);
  }

  public void useOutputNoSetpoint(double currErr) {
    SmartDashboard.putNumber("measured error", currErr);
  }

  public void setTolerance(double tolerance) {
    getController().setTolerance(tolerance);
  }



  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getController().getPositionError();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    
  }
}