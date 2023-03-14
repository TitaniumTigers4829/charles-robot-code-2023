// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmSubsystem extends Subsystem {

  /** 
   * Gets the arm's extension in meters.
   */
  public double getCurrentExtension();

  /** 
   * Sets the arm's extension in meters.
   */
  public void setExtension(double extension);

  /**
   * Sets the arm's extension to 0.
   */
  public void retractArm();

  /**
   * Resets the extension motor's encoder to 0.
   */
  public void resetExtensionEncoder();

  /** 
   * Returns the angle, in degrees, of the arm (0 being straight down).  
   */
  public double getAngle();

  /** 
   * Sets the arm angle in degrees (0 being straight down).
   */
  public void goToAngle(double desiredAngle);

  /**
   * Locks the extension solenoid
   */
  public void lockExtensionSolenoid();
  
  /**
   * Unlocks the extension solenoid
   */
  public void unlockExtensionSolenoid();

  /**
   * Sets the speed from -1 to 1 of the rotation motors.
   */
  public void setRotationSpeed(double speed);

  /**
   * Returns the speed of the extension motor.
   */
  public double getCurrentExtensionSpeed();

  /**
   * Sets the speed from -1 to 1 of the rotation motors.
   */
  public void setCurrentExtensionSpeed(double speed);

  /**
   * Resets the PID controller for the extension motor.
   */
  public void resetExtensionController();

  /**
   * Returns true if the extension motor is stalling.
   */
  public boolean isExtensionMotorStalling();

  /**
   * Sets the neutral mode of the extension motor.
   */
  public void setExtensionMotorNeutralMode(NeutralMode neutralMode);

  }