// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pigeon2Subsystem;

public class PathForward extends CommandBase {
  /** Creates a new PathForward. */
  private Drivetrain m_Drivetrain;
  private Pigeon2Subsystem m_Pigeon2;

  private boolean finish = false;

  public PathForward(Drivetrain drivetrain, Pigeon2Subsystem pigeon) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Drivetrain = drivetrain;
    m_Pigeon2 = pigeon;
    addRequirements(m_Drivetrain, m_Pigeon2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forward(-0.6, 1);
  }

  private void forward(double power, double scale) {
    if (Math.abs(m_Pigeon2.getRoll()) <= Constants.MAX_INCLINATION_ANGLE) {
      m_Drivetrain.setMotorsVelocity(power, power, scale);
    }
    else {
        m_Drivetrain.setStartDist(m_Drivetrain.getConvertedToMeters(m_Drivetrain.getSensorValues()));
        finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setMotors(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false || finish;
  }
}
