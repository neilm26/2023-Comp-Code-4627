// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NetworktablesUpdated extends SubsystemBase {
  /** Creates a new NetworktablesUpdated. */
  public GenericEntry balance_kp_entry, balance_ki_entry, balance_kd_entry, 
            ramp_kp_entry, ramp_ki_entry, ramp_kd_entry, ka_entry, kv_entry;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
  public void BuildWidget() {
    balance_kp_entry = tab.addPersistent("balancekP", Constants.PIGEON_KP).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.3)).getEntry();
    balance_ki_entry = tab.addPersistent("balancekI", Constants.PIGEON_KI).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.03)).getEntry();      
    balance_kd_entry = tab.addPersistent("balancekD", Constants.PIGEON_KD).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.05)).getEntry(); 


    ramp_kp_entry = tab.addPersistent("rampkP", Constants.RAMP_KP).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.3)).getEntry();
    ramp_ki_entry = tab.addPersistent("rampkI", Constants.RAMP_KI).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.03)).getEntry();      
    ramp_kd_entry = tab.addPersistent("rampkD", Constants.RAMP_KD).
          withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",0.05)).getEntry(); 
  }

  public NetworktablesUpdated() {} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
