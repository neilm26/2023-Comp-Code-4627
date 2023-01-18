// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.NetworktablesUpdated;
import frc.robot.Subsystems.Pigeon;
import frc.robot.Subsystems.Pigeon2Subsystem;

public class Tuner extends CommandBase {
  /** Creates a new Tuner. */
  private final Drivetrain m_Drivetrain;
  private final Pigeon2Subsystem m_Pigeon;
  private final NetworktablesUpdated m_table;

  public Tuner(Drivetrain drivetrain, Pigeon2Subsystem pigeon, NetworktablesUpdated table) {
    m_Drivetrain = drivetrain;
    m_Pigeon = pigeon;
    m_table = table;

    addRequirements(drivetrain, pigeon, table);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.getController().setTolerance(Constants.PIGEON_TOLERANCE); //change to Constants.TOLERANCE
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    m_Drivetrain.getController().getController().setPID(m_table.kp_entry.getDouble(0)
          ,m_table.ki_entry.getDouble(0),m_table.kd_entry.getDouble(0));

    
    SmartDashboard.putNumber("calculated: ", m_Drivetrain.getController().returnCalc(m_Pigeon.getRoll(), m_Pigeon.getSetpoint()));
    SmartDashboard.putNumber("PIGEON MEASURE: ", m_Drivetrain.getController().getMeasurement());
    //blue = ideal acceleration
    //black = tracked acceleration
    //red = tracked pitch angle
    double right_power = Utilities.constrain(m_Drivetrain.getController().returnCalc(m_Pigeon.getRoll(), m_Pigeon.getSetpoint()),-1,1);
    double power = Utilities.constrain(m_Drivetrain.getController().returnCalc(m_Pigeon.updateGravityVectors()[0], right_power), -1, 1);

    m_Drivetrain.setMotors(right_power, right_power);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
