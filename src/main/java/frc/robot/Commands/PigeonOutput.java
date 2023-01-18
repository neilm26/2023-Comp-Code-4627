// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Pigeon;

public class PigeonOutput extends CommandBase {
  /** Creates a new PigeonOutput. */
  private final Pigeon m_Pigeon;
  public PigeonOutput(Pigeon pigeon) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Pigeon = pigeon;

    addRequirements(pigeon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  double[] tmp = new double[3];

  @Override
  public void execute() {
    tmp[1] =  m_Pigeon.cancelNoise(9.81 * Math.sin(m_Pigeon.getRoll()),10); //angular acceleration
    tmp[0] = m_Pigeon.cancelNoise(m_Pigeon.getRate()[0], 3); //angular velocity
    tmp[2] = m_Pigeon.getRoll(); //get inclination
    SmartDashboard.putNumber("graph heading: ", m_Pigeon.getRoll());
    SmartDashboard.putNumberArray("inclination: ", tmp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
