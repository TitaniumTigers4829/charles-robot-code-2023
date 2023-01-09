// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limb.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimbSubsystem;

public class ToggleClaw extends CommandBase {

  private LimbSubsystem limbSubsystem;

  public ToggleClaw(LimbSubsystem limbSubsystem) {
    this.limbSubsystem = limbSubsystem;
    addRequirements(limbSubsystem);
  }


  @Override
  public void initialize() {
    limbSubsystem.ToggleClaw();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
