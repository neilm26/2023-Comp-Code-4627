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

  private Pigeon2Subsystem pigeon2;

  private double distChange = 0;
  private boolean alreadyUpdated = false;
  
  public DriveControls(Drivetrain drive, Supplier<Double> joystickX, Supplier<Double> righttrigger, 
  Supplier<Double> lefttrigger, Supplier<Boolean> yButton, Pigeon2Subsystem pigeon2) {
    addRequirements(drive);
    this.drive = drive;
    this.joystickX = joystickX;
    this.righttrigger = righttrigger;
    this.lefttrigger = lefttrigger;
    this.yButton = yButton;
    this.pigeon2 = pigeon2;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alreadyUpdated = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = (lefttrigger.get() - righttrigger.get());

    double turn = Math.cbrt(joystickX.get());
    //drive.setMotors(speed + turn, speed - turn, Constants.SPEED_MULTIPLIER);
    
    drive.setMotorsVelocity(speed + turn, speed - turn, Constants.SPEED_MULTIPLIER);
    
    if (Math.abs(pigeon2.getRoll() - pigeon2.getSetpoint()) >= 8) {
      SmartDashboard.putBoolean("onRamp", true);
      if (alreadyUpdated == false) { //CLEANUP!
        drive.setStartDist(drive.getConvertedToMeters(drive.getSensorValues()));
        alreadyUpdated = true;
      }
      distChange = drive.getConvertedToMeters(drive.getSensorValues()) - 
                  drive.getStartDist();

      SmartDashboard.putNumber("change: ", distChange);
    }
    else {
      SmartDashboard.putBoolean("onRamp", false);
      //drive.resetEncoders();
      distChange = 0;
      alreadyUpdated = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setStartDist(distChange);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
