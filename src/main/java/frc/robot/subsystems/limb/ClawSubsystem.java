package frc.robot.subsystems.limb;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ClawSubsystem extends Subsystem {

  /** 
   * Closes the claw.
   */
  public void close();

  /** 
   * Releases the contents of the claw. 
   */
  public void open();

  /** 
   * Returns true if the claw is closed. 
   */
  public boolean getClawClosed();

  /** 
   * Sets the motor to the specified speed (from 0 to 1). 
   */
  public void setMotorSpeed(double speed);

  /** 
   * Returns the angle, in radians, of the wrist.
   */
  public double getWristAngle();

  /** Sets the target angle of the wrist.
   * @param angle (radians)
   */
  void setWristAngle(double angle);
}
