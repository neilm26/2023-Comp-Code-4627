// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrain;

public class Tuner extends CommandBase {
  /** Creates a new Tuner. */
  private final Drivetrain m_Drivetrain;

  private double m_startDist;

  
  public Tuner(Drivetrain drivetrain) {
    m_Drivetrain = drivetrain;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Drivetrain.setToFMode(RangingMode.Long);

    m_startDist = m_Drivetrain.getStartDist();
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    climb(m_Drivetrain.shfVals().get("rampkP"), 
        m_Drivetrain.shfVals().get("rampkI"), m_Drivetrain.shfVals().get("rampkD"));
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setMotorsVelocity(0, 0, 0);
  }

  private void climb(double m_kP, double m_kI, double m_kD) {

    m_Drivetrain.getController().getController().setPID(m_kP, m_kI, m_kD); 

    double filtered = m_Drivetrain.getController().
        returnCalc(m_startDist-m_Drivetrain.getConvertedToMeters(m_Drivetrain.getSensorValues()), 
        Constants.BALANCE_LEN);

    SmartDashboard.putNumber("filtered: ", filtered);
    SmartDashboard.putNumber("error: ", m_Drivetrain.getController().getMeasurement());

    double power = Utilities.constrain(filtered,-1,1);
    m_Drivetrain.setMotorsVelocity(power, power, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Drivetrain.getController().getMeasurement()) <= Constants.DRIVE_POS_ERR_TOL;
  }
}
