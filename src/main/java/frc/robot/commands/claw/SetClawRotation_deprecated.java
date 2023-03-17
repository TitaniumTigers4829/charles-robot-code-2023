// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawRotation_deprecated extends CommandBase {
  
  private ClawSubsystem clawSubsystem;
  private int wristAngle;

  public SetClawRotation_deprecated(ClawSubsystem clawSubsystem, int wristAngle) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.clawSubsystem);
    this.wristAngle = wristAngle;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawSubsystem.setWristPosition(wristAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
