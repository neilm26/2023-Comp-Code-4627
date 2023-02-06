// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pigeon2Subsystem extends SubsystemBase {
  /** Creates a new Pigeon2. */
  private final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(Constants.PIGEON_ID);

  private double setpoint;
  private double[] getRawGyroAngles = new double[3];
  private double[] getGravityVectors = new double[3];
  private double[] DimQuad = new double[4];
  private short[] biasedAcc = new short[3];
  private double[] accumGyroReadings = new double[3];

  public Pigeon2Subsystem() {}

  public double cancelNoise(double input, int taps) {
    LinearFilter AccelFilter = LinearFilter.movingAverage(taps);
    return AccelFilter.calculate(input);
  }

  // public  Matrix calculateWithLQR(Matrix curr, Matrix next) {
  //   return balance_LQR.calculate(curr, next);
  // }

  


  public void configure() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch=1;
    pigeon2.configAllSettings(config);
  }

  public double getError(double curr, double target) {
    return Math.abs(curr-target);
  }

  public double[] getRate() {
    pigeon2.getRawGyro(getRawGyroAngles);

    return getRawGyroAngles;
  }

  public double[] updateGravityVectors() {
    pigeon2.getGravityVector(getGravityVectors);
    //[0] returns pitch
    return getGravityVectors;
  }

  public double[] getQuad() {
    pigeon2.get6dQuaternion(DimQuad);

    return DimQuad;
  }

  public short[] getBiasedMagn() {
    pigeon2.getBiasedAccelerometer(biasedAcc);

    return biasedAcc;
  }

  public double[] getAccum() {
    pigeon2.getAccumGyro(accumGyroReadings);

    return accumGyroReadings;
  }
  

  public double getPitch() {
    return pigeon2.getPitch();
  }
  public double getYaw() {
    return pigeon2.getYaw();
  }
  public double getRoll() {
    return pigeon2.getRoll();
  }

  public void setYaw(double ang) {
    pigeon2.setYaw(ang);
  }

  public void setSetpoint(double p) {
    setpoint = p;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getAccelRoll(double accel_x, double accel_y, double accel_z) {
    return -Math.atan2(accel_z, Math.sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0 / Math.PI;
  }

  public double getAccelPitch(double y, double z) {
    return Math.atan2(y,z) * 180 / Math.PI;
  }

  public double getKalmanPitch(double accelPitch, double gyro_y) {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
