package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ClawSubsystem extends Subsystem {

  /** 
   * Closes the claw.
   */
  void close();

  /** 
   * Opens the claw.
   */
  void open();

  /** 
   * Returns true if the claw is closed. 
   */
  boolean isClawClosed();

  /** 
   * Sets the wheel motors to the specified speed (from -1 to 1). 
   */
  void setIntakeSpeed(double speed);

  /**
   * Sets the wrist motor speed (from -1 to 1).
   */
  void setWristMotorSpeed(double speed);
  /** 
   * Returns the angle, in radians, of the wrist (0 meaning the claw is
   * parallel with the ground).
   */
  double getWristAngle();

  /**
   * Resets the wrist's encoder position to the minimum position.
   */
  void zeroWristEncoder();

  /**
   * Sets the position of the wrist in radians.
   */
  void setWristPosition(double angle);
  
  /**
   * Returns true if the claw is in cone mode, false for cube mode.
   */
  boolean isConeMode();

  /**
   * Switches the cargo mode from cone to cube or vice versa.
   */
  void switchCargoMode();
  
  /**
   * Sets the cargo mode to cone.
   */
  void setCargoModeCone();

  /**
   * Sets the cargo mode to cone.
   */
  void setCargoModeCube();

  /**
   * Returns true if the claw is in manual control mode.
   */
  boolean isManualControl();

  /**
   * Toggles the claw control mode between manual and auto
   */
  void toggleControlMode();
}
