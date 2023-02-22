// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public interface ArmSubsystem {
  public void closeClaw();
  public void openClaw();
  public void extendArm();
  public void closeArm();
  public void setDesiredWristRotation(double rotation);
}
