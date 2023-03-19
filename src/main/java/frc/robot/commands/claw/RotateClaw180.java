// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;

public class RotateClaw180 extends CommandBase {
  private ClawSubsystem clawSubsystem;
  private double initialRotation;
  /** Creates a new RotateClaw180. */
  public RotateClaw180(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialRotation = clawSubsystem.getWristAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initialRotation > 90) {
      clawSubsystem.setWristPosition(0);
    } else {
      clawSubsystem.setWristPosition(180);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double desired = (initialRotation > 90 ? 0 : 180);
    return (Math.abs(clawSubsystem.getWristAngle() - desired) < 3.0);
  }
}
