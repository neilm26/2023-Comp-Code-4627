// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NetworktablesUpdated extends SubsystemBase {
  /** Creates a new NetworktablesUpdated. */
  public GenericEntry entry;

  public void BuildWidget() {
    entry = Shuffleboard.getTab("Drive").addPersistent("kP", Constants.PIGEON_KP).
          withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  }

  public NetworktablesUpdated() {} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
