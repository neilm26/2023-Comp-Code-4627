// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworktablesUpdated extends SubsystemBase {
  /** Creates a new NetworktablesUpdated. */
  private final NetworkTableInstance table = NetworkTableInstance.getDefault();

  private final NetworkTable dataTable = table.getTable("Drive");

  public DoubleSubscriber GetDoubleEntry(String name) {
    DoubleSubscriber get = dataTable.getDoubleTopic(name).subscribe(0);

    return get;
  }

  public DoublePublisher SetDoubleEntry(String name, double defaultVal) {
    DoublePublisher set = dataTable.getDoubleTopic(name).publish();

    return set;
  }

  public GenericEntry BuildWidget() {
    return Shuffleboard.getTab("Drive").addPersistent("Max Speed", 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  }

  public NetworktablesUpdated() {} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
