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
  double getExtension();

  /** 
   * Sets the arm's extension in meters.
   */
  void setExtension(double extension);

  /**
   * Resets the extension motor's encoder to 0.
   */
  void resetExtensionEncoder();

  /** 
   * Returns the angle, in degrees, of the arm (0 being straight down).  
   */
  double getRotation();

  /** 
   * Sets the arm angle in degrees (0 being straight down).
   */
  void setRotation(double desiredAngle);

  /**
   * Sets the rotation motors' encoders to that of the cancoder.
   */
  void syncRotationEncoders();

  /**
   * Locks the extension solenoid
   */
  void lockExtensionSolenoid();
  
  /**
   * Unlocks the extension solenoid
   */
  void unlockExtensionSolenoid();

  /**
   * Returns the rotation speed of the arm in degrees per second.
   */
  double getRotationSpeed();

  /**
   * Sets the speed from -1 to 1 of the rotation motors.
   */
  void setRotationSpeed(double speed);

  /**
   * Returns the speed of the extension motor.
   */
  double getExtensionSpeed();

  /**
   * Sets the speed from -1 to 1 of the rotation motors.
   */
  void setExtensionSpeed(double speed);

  /**
   * Returns the torque on the arm caused by gravity.
   */
  double getTorqueFromGravity();

  /**
   * Sets the neutral mode of the extension motor.
   */
  void setExtensionMotorNeutralMode(NeutralMode neutralMode);

  /**
   * Resets the PID controller for the extension motor.
   */
  void resetExtensionController();

  }