// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.ClawSubsystem;

public class RunClaw extends CommandBase {
  private ClawSubsystem clawSubsystem;
  private double speed;

  public RunClaw(ClawSubsystem clawSubsystem, double speed) {
    this.clawSubsystem = clawSubsystem;
    this.speed = speed;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawSubsystem.setIntakeSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
