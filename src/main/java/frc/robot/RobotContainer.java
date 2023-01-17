// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.DriveControls;
import frc.robot.Commands.PigeonTiltingPID;
import frc.robot.Commands.Tuner;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.NetworktablesUpdated;
import frc.robot.Subsystems.PID;
import frc.robot.Subsystems.Pigeon;

/** Add your docs here. */
public class RobotContainer {

 
  private OI m_OI = new OI();

  //changeable vars

  // The robot's subsystems and commands are defined here...

  // Subsystems
  private Drivetrain m_drivetrain = new Drivetrain(new PID(Constants.PIGEON_KP, Constants.PIGEON_KI, Constants.PIGEON_KD));
  private Pigeon m_Pigeon = new Pigeon();
  private NetworktablesUpdated m_table = new NetworktablesUpdated();

  //entry acting funny.
  private Command m_Tune = new Tuner(m_drivetrain, m_Pigeon, m_table);


  
  
  public RobotContainer() {
    configureButtonBindings();
  }



  public void configureButtonBindings() {
  
    // configures all the button bindings for the robot
    m_table.BuildWidget();

    // setup default commands
    this.m_drivetrain.setDefaultCommand(getDriverControls());
    m_OI.dButtonB.toggleOnTrue(m_Tune);
    m_OI.dButtonX.onTrue(new InstantCommand(() -> m_Pigeon.setSetpoint(m_Pigeon.getRoll())));

    LiveWindow.enableAllTelemetry();
  }

  public Command getDriverControls() {
    return new DriveControls(m_drivetrain, () -> m_OI.getDriverRawAxis(Constants.LEFT_STICK_X),
        () -> m_OI.getDriverRawAxis(Constants.RIGHT_TRIGGER),
        () -> m_OI.getDriverRawAxis(Constants.LEFT_TRIGGER), () -> m_OI.getDriverButton(Constants.BUTTON_Y));
  }

  public Command getPigeonReadings() {
    return new PigeonTiltingPID(m_Pigeon, new PID(Constants.PIGEON_KP, Constants.PIGEON_KI, Constants.PIGEON_KD));
  }
}
