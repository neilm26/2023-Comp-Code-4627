// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NetworktablesUpdated extends SubsystemBase {
  /** Creates a new NetworktablesUpdated. */
  public GenericEntry kp_entry,ki_entry,kd_entry;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
  public void BuildWidget() {
    kp_entry = tab.addPersistent("kP", Constants.PIGEON_KP).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.3)).getEntry();
    ki_entry = tab.addPersistent("kI", Constants.PIGEON_KI).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.03)).getEntry();      
    kd_entry = tab.addPersistent("kD", Constants.PIGEON_KD).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.05)).getEntry(); 
  }

  public NetworktablesUpdated() {} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
