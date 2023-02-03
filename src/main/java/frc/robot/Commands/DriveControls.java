// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pigeon2Subsystem;

public class DriveControls extends CommandBase {
  /** Creates a new DriveControls. */
  private Drivetrain drive;
  private Supplier<Double> joystickX;
  private Supplier<Double> righttrigger;
  private Supplier<Double> lefttrigger;
  private Supplier<Boolean> yButton;

  private double speed;
  private double turn;
  
  public DriveControls(Drivetrain drive, Supplier<Double> joystickX, Supplier<Double> righttrigger, 
  Supplier<Double> lefttrigger, Supplier<Boolean> yButton) {
    addRequirements(drive);
    this.drive = drive;
    this.joystickX = joystickX;
    this.righttrigger = righttrigger;
    this.lefttrigger = lefttrigger;
    this.yButton = yButton;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = (lefttrigger.get() - righttrigger.get());

    turn = Math.cbrt(joystickX.get());
   // drive.setMotors(speed + turn, speed - turn, Constants.SPEED_MULTIPLIER);
    
    drive.setMotorsVelocity(speed + turn, speed - turn, Constants.SPEED_MULTIPLIER); // change back speed multiplier
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
