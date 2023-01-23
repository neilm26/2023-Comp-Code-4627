// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.FeedForward;
import frc.robot.Subsystems.NetworktablesUpdated;
import frc.robot.Subsystems.Pigeon2Subsystem;

public class Tuner extends CommandBase {
  /** Creates a new Tuner. */
  private final Drivetrain m_Drivetrain;
  private final Pigeon2Subsystem m_Pigeon;
  private final NetworktablesUpdated m_table;
  private final FeedForward m_FeedForward;

  public Tuner(Drivetrain drivetrain, Pigeon2Subsystem pigeon, NetworktablesUpdated table, FeedForward feedForward) {
    m_Drivetrain = drivetrain;
    m_Pigeon = pigeon;
    m_table = table;
    m_FeedForward = feedForward;

    addRequirements(drivetrain, pigeon, table, feedForward);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.getController().setTolerance(Constants.PIGEON_TOLERANCE, 15); //change to Constants.TOLERANCE
  }

  // Called every time the scheduler runs while the command is scheduled.
  double[] tmp = new double[3];

  @Override
  public void execute() {
    double m_kP = m_table.kp_entry.getDouble(0);
    double m_kI = m_table.ki_entry.getDouble(0);
    double m_kD = m_table.kd_entry.getDouble(0);

    double m_kV = m_table.kv_entry.getDouble(0);
    double m_kA = m_table.ka_entry.getDouble(0);
    m_Drivetrain.getController().getController().setPID(m_kP, m_kI, m_kD);
    m_FeedForward.tuneFF(m_kV, m_kA);

    
    SmartDashboard.putNumber("PIGEON MEASURE: ", m_Drivetrain.getController().getMeasurement());
    //blue = ideal acceleration
    //black = tracked acceleration
    //red = tracked pitch angle
    //double filtered = m_Drivetrain.getController().returnCalc(m_Pigeon.getRoll(), m_Pigeon.getSetpoint());

    //Important: 2 taps for a small bot, 6 taps for snoopy
    double filtered = m_Drivetrain.getController().returnCalc(m_Pigeon.cancelNoise(m_Pigeon.getRoll(),6), m_Pigeon.getSetpoint());
    SmartDashboard.putNumber("filtered: ", filtered);
    double right_power = Utilities.constrain(filtered,-1,1);
    //double power = Utilities.constrain(m_Drivetrain.getController().returnCalc(m_Pigeon.updateGravityVectors()[0], right_power), -1, 1);
    
    m_Drivetrain.setMotorsVelocity(right_power, right_power, 1);

    //tmp[1] =  m_Pigeon.cancelNoise(m_Pigeon.getRoll(),3); //angular acceleration
    tmp[0] = m_Pigeon.getRoll(); //angular velocity ; x = [0], y = [1], z = [2]
    tmp[1] = m_Pigeon.getRate()[0];
    
    SmartDashboard.putNumber("graph heading: ", m_Pigeon.getRoll());
    SmartDashboard.putNumberArray("unfiltered vs. filtered: ", tmp); //broadcast all three readings, good for snapshotting
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
