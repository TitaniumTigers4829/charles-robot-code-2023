// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawRotation extends CommandBase {

  private final ClawSubsystem clawSubsystem;
  private final double rotation;

  public SetClawRotation(ClawSubsystem clawSubsystem, double rotation) {
    this.clawSubsystem = clawSubsystem;
    this.rotation = rotation;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    clawSubsystem.setWristPosition(rotation);
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.setWristMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
