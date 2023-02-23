// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmSubsystem extends Subsystem {
  /** Extends the arm. */
  void extendArm();
  /** Retracts the arm. */
  void retractArm();
  /** Toggles the arm's extension state. */
  void toggleArm();
  /** Returns true if the arm is currently extended. */
  boolean getArmExtended();


  /** Returns the angle, in radians, of the swinging arm. */
  double getAngle();

  /** Sets the target angle of the swinging arm.
   * @param angle (radians)
   */
  void goToAngle(double angle);
}
