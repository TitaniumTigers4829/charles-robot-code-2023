// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Neo550SubsystemImpl implements Neo550Subsystem {

  private final CANSparkMax neo550;
  private final SparkMaxPIDController pid;
  private final RelativeEncoder encoder;

  /** Creates a new Neo550SubsystemImpl. */
  public Neo550SubsystemImpl(int deviceID, boolean inverted) {
    neo550 = new CANSparkMax(deviceID, MotorType.kBrushless);
    neo550.setInverted(inverted);
    encoder = neo550.getEncoder();
    pid = neo550.getPIDController();
    pid.setFeedbackDevice(encoder);
    pid.setOutputRange(-1, 1);
    setFeedForward(0);
  }

  /** Creates a new Neo550SubsystemImpl with the given PID values. */
  public Neo550SubsystemImpl(int deviceID, boolean inverted, double p, double i, double d, double gain) {
    neo550 = new CANSparkMax(deviceID, MotorType.kBrushless);
    neo550.setInverted(inverted);
    encoder = neo550.getEncoder();
    pid = neo550.getPIDController();
    pid.setFeedbackDevice(encoder);
    setPID(p, i, d, -1, 1);
    setFeedForward(gain);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setSpeed(double speed) {
    neo550.set(speed);
  }

  @Override
  public void setIdleMode(IdleMode mode) {
    neo550.setIdleMode(mode);
  }
  
  @Override
  public void setPID(double p, double i, double d) {
    pid.setP(p);
    pid.setI(i);
    pid.setD(d);
  }

  @Override
  public void setPID(double p, double i, double d, double minRange, double maxRange) {
    setPID(p, i, d);
    pid.setOutputRange(minRange, maxRange);
  }

  @Override
  public void goToPosition(double position) {
    pid.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setFeedForward(double gain) {
    pid.setFF(gain);
  }

}
