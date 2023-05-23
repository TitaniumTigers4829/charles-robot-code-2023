// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeSubsystem extends Subsystem {
  /**
   * Gets out of the way of the arm
   */
  public void getOutOfTheWay();

  /**
   * Intakes
   */
  public void intake();

  /**
   * Manual control for the intake
   * @param speed1 Speed for motor 1
   * @param speed2 Speed for motor 2
   * @param extended Whether or not to extend
   */
  public void manual(double speed1, double speed2, boolean extended);

  /**
   * Gets rid of cargo, either by passing it to the claw or shooting it
   */
  public void getRidOfCargo();
}
