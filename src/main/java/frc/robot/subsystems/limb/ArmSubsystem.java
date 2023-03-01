// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmSubsystem extends Subsystem {

  /** 
   * Gets the arm's extension 0 to 1.
   */
  public boolean getExtension();

  /** 
   * Sets the arm's extension from 0 to 1.
   */
  public void setExtension(double armExtension);

  /** 
   * Returns the angle, in radians, of the arm (0 being straight down).  
   */
  public double getAngle();

  /** 
   * Sets the arm angle in radians (0 being straight down).
   */
<<<<<<< HEAD
  public void goToAngle(double setpoint, double currentPosition, double currentVelocity);
=======
  public void goToAngle(double armRotation);
>>>>>>> 00342b6cc51417ee334f008e726f6116c13f4423

}
