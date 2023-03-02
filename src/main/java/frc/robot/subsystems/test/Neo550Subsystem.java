package frc.robot.subsystems.test;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Neo550Subsystem extends Subsystem {
  
  /** 
   * Sets the speed of the Neo550.
   */
  public void setSpeed(double speed);
  /** 
   * Sets the idle mode of the Neo550.
   */
  public void setIdleMode(IdleMode mode);
  /**
   * Sets the internal PID values.
   */
  public void setPID(double p, double i, double d);
  /**
   * Sets the internal PID values.
   * @param minRange the minimum value in the output range
   * @param maxRange the maximum value in the output range
   */
  public void setPID(double p, double i, double d, double minRange, double maxRange);
  /**
   * Goes to the specified position, in encoder units.
   */
  public void goToPosition(double position);
  /** 
   * Sets the FeedForward gain constants.
   */
  public void setFF(double gain);

}
