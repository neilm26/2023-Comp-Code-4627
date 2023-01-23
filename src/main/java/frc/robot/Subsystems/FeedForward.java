// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeedForward extends SubsystemBase {
  /** Creates a new FeedForward. */
  SimpleMotorFeedforward drivFeedforward = new SimpleMotorFeedforward(Constants.FEEDFORWARD_KS, 
                        Constants.FEEDFORWARD_KV, Constants.FEEDFORWARD_KA);   

  public FeedForward() {
    //calibrate other min-max vals
  }


  public double FFCalculate(double velocity, double acceleration) {
    return drivFeedforward.calculate(velocity, acceleration);
  }

  public void tuneFF(double inp_kv, double inp_ka) {
    drivFeedforward = new SimpleMotorFeedforward(Constants.FEEDFORWARD_KS, inp_kv, inp_ka);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
