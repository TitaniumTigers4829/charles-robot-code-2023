package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ClawSubsystem extends Subsystem {

  /** 
   * Closes the claw.
   */
  public void close();

  /** 
   * Opens the claw.
   */
  public void open();

  /** 
   * Returns true if the claw is closed. 
   */
  public boolean isClawClosed();

  /** 
   * Sets the wheel motors to the specified speed (from -1 to 1). 
   */
  public void setIntakeSpeed(double speed);

  /**
   * Sets the wrist motor speed (from -1 to 1).
   */
  public void setWristMotorSpeed(double speed);
  /** 
   * Returns the angle, in radians, of the wrist (0 meaning the claw is
   * parallel with the ground).
   */
  public double getWristAngle();

  /**
   * Resets the wrist's encoder position to the minimum position.
   */
  public void zeroWristEncoder();

  /**
   * Sets the position of the wrist in radians.
   */
  public void setWristPosition(double angle);
  
}
