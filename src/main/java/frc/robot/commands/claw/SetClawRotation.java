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
  /** Creates a new SetClawRotation. */
  public SetClawRotation(ClawSubsystem clawSubsystem, double desiredRotation) {
    this.clawSubsystem = clawSubsystem;
    this.desiredRotation = desiredRotation;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSubsystem.goToWristAngle(desiredRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(clawSubsystem.getWristAngle() - desiredRotation) < ClawConstants.WRIST_ROTATION_ACCEPTABLE_ERROR;
  }
}
