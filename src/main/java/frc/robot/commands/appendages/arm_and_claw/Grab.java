// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.appendages.arm_and_claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class Grab extends CommandBase {

  private ArmSubsystem armSubsystem;

  public Grab(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }


  @Override
  public void initialize() {
    armSubsystem.CloseClaw();
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.OpenClaw();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
