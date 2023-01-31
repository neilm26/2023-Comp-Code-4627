// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pigeon2Subsystem;

public class Correcter extends CommandBase {
  /** Creates a new Correcter. */
  private final Drivetrain m_Drivetrain;
  private final Pigeon2Subsystem m_Pigeon;

  private boolean cancelCommand = false;
  
  public Correcter(Drivetrain drivetrain, Pigeon2Subsystem pigeon) {
    m_Drivetrain = drivetrain;
    m_Pigeon = pigeon;
    addRequirements(drivetrain, pigeon);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //possibly unneeded
    m_Drivetrain.getController().setTolerance(Constants.PIGEON_TOLERANCE, 5); //change to Constants.TOLERANCE
  }

  double[] tmp = new double[3];

  private boolean abort(double max_thresh_ang, double curr_ang) {
    return curr_ang >= max_thresh_ang;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cancelCommand = abort(Constants.MAX_INCLINATION_ANGLE, Math.abs(m_Pigeon.getRoll()));

    tmp[0] = m_Pigeon.getRoll(); //angular velocity ; x = [0], y = [1], z = [2]
    tmp[1] = m_Pigeon.getRate()[0];
    
    SmartDashboard.putNumberArray("unfiltered vs. filtered: ", tmp); //broadcast all three readings, good for snapshotting
    SmartDashboard.putBoolean("abort: ", cancelCommand);
    
    cycleBalance(m_Drivetrain.shfVals().get("balancekP"), 
        m_Drivetrain.shfVals().get("balancekI"), m_Drivetrain.shfVals().get("balancekD"));
  }

  public void cycleBalance(double m_kP, double m_kI, double m_kD) {
    m_Drivetrain.getController().getController().setPID(m_kP, m_kI, m_kD); 

    double pigeonFiltered = m_Drivetrain.getController().returnCalc(m_Pigeon.cancelNoise(m_Pigeon.getRoll(),6), m_Pigeon.getSetpoint());

    double power = Utilities.constrain(pigeonFiltered,-1,1);
    m_Drivetrain.setMotorsVelocity(power, power, 1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false || cancelCommand;
  }
}
