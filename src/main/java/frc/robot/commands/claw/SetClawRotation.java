// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawRotation extends CommandBase {
  
  private ClawSubsystem clawSubsystem;
  private double desiredRotation;

  public SetClawRotation(ClawSubsystem clawSubsystem, double desiredRotation) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.clawSubsystem);
    this.desiredRotation = desiredRotation;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawSubsystem.goToWristAngle(desiredRotation);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(desiredRotation - clawSubsystem.getWristAngle()) < ClawConstants.WRIST_ROTATION_ACCEPTABLE_ERROR;
  }
}
