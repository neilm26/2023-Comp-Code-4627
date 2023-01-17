// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pigeon extends SubsystemBase {
  /** Creates a new Pigeon. */
  private final WPI_PigeonIMU pigeonIMU = new WPI_PigeonIMU(Constants.PIGEON_ID);
  private final Accelerometer accelerometer = new BuiltInAccelerometer();

  private final LinearSystem<N2, N1, N1> pigeonPlant = LinearSystemId.identifyPositionSystem( 
                                              Constants.FEEDFORWARD_KV, Constants.FEEDFORWARD_KA);

  private double setpoint;
  
  public Pigeon() {
  }

  public ArrayList<Double> returnAngleRate() {
    ArrayList<Double> accel_arr = new ArrayList<>(Arrays.asList(accelerometer.getX(), accelerometer.getY(), accelerometer.getZ()));
    return accel_arr;
  }

  public double getError(double curr, double target) {
    return Math.abs(curr-target);
  }

  public double getPitch() {
    return pigeonIMU.getPitch();
  }
  public double getYaw() {
    return pigeonIMU.getYaw();
  }
  public double getRoll() {
    return pigeonIMU.getRoll();
  }

  public void setYaw(double ang) {
    pigeonIMU.setYaw(ang);
  }

  public void setSetpoint(double p) {
    setpoint = p;
  }

  public double getSetpoint() {
    return setpoint;
  }


  public void setFusedHeading(double offset) {
    pigeonIMU.setFusedHeading(offset);
  }

  public double getAccelPitch(double accel_x, double accel_y, double accel_z) {
    return -Math.atan2(accel_z, Math.sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0 / Math.PI;
  }

  public double getKalmanPitch(double accelPitch, double gyro_y) {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}