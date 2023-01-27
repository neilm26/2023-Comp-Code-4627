// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import java.util.HashMap;

import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.NetworktablesUpdated;
import frc.robot.Subsystems.Pigeon2Subsystem;

public class Tuner extends CommandBase {
  /** Creates a new Tuner. */
  private final Drivetrain m_Drivetrain;
  private final Pigeon2Subsystem m_Pigeon;
  private final NetworktablesUpdated m_table;

  private boolean cancelCommand = false;

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
    m_Drivetrain.getController().setTolerance(Constants.PIGEON_TOLERANCE, 15); //change to Constants.TOLERANCE
    m_Drivetrain.setToFMode(RangingMode.Long);
  }

  // Called every time the scheduler runs while the command is scheduled.
  double[] tmp = new double[3];

  private boolean abort(double max_thresh_ang, double curr_ang) {
    return curr_ang >= max_thresh_ang;
  }

  @Override
  public void execute() {
    //Important: 2 taps for a small bot, 6 taps for snoopy

    cancelCommand = abort(Constants.MAX_INCLINATION_ANGLE, Math.abs(m_Pigeon.getRoll()));

    cycleBalance(shfVals().get("kP"), shfVals().get("kI"), shfVals().get("kD"));

    tmp[0] = m_Pigeon.getRoll(); //angular velocity ; x = [0], y = [1], z = [2]
    tmp[1] = m_Pigeon.getRate()[0];
    
    SmartDashboard.putNumberArray("unfiltered vs. filtered: ", tmp); //broadcast all three readings, good for snapshotting
    SmartDashboard.putNumberArray("accum: ", m_Pigeon.getAccum());

    SmartDashboard.putBoolean("abort: ", cancelCommand);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  private HashMap<String, Double> shfVals() {
    double m_kP = m_table.kp_entry.getDouble(0);
    double m_kI = m_table.ki_entry.getDouble(0);
    double m_kD = m_table.kd_entry.getDouble(0);

    double m_kV = m_table.kv_entry.getDouble(0);
    double m_kA = m_table.ka_entry.getDouble(0);

    HashMap<String, Double> dict = new HashMap<String, Double>();

    dict.put("kP", m_kP);
    dict.put("kI", m_kI);
    dict.put("kD", m_kD);

    return dict;
  }

  private void cycleBalance(double m_kP, double m_kI, double m_kD) {
    m_Drivetrain.getController().getController().setPID(m_kP, m_kI, m_kD); 

    double filtered = m_Drivetrain.getController().returnCalc(m_Pigeon.cancelNoise(m_Pigeon.getRoll(),6), m_Pigeon.getSetpoint());
    SmartDashboard.putNumber("filtered: ", filtered);
    double power = Utilities.constrain(filtered,-1,1);
    m_Drivetrain.setMotorsVelocity(power, power, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false || cancelCommand;
  }
}
