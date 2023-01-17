// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.PID;
import frc.robot.Subsystems.Pigeon;

public class PigeonTiltingPID extends CommandBase {
  /** Creates a new PigeonTiltingPID. */
  private Pigeon m_Pigeon;
  private PID m_PID;
  public PigeonTiltingPID(Pigeon pigeon, PID p) {
    m_Pigeon = pigeon; m_PID = p;
    addRequirements(pigeon, p);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Pigeon.setYaw(0);
    m_PID.setTolerance(Constants.PIGEON_TOLERANCE); //change to Constants.TOLERANCE
    SmartDashboard.putNumber("Checking to confirm: ", m_PID.getController().getP());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      SmartDashboard.putNumber("calculated: ", m_PID.returnCalc(m_Pigeon.getYaw(), 0));
      SmartDashboard.putNumber("PID MEASUREMENT:", m_PID.getMeasurement());
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