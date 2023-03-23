// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ToggleClaw extends InstantCommand {
  private ClawSubsystem clawSubsystem;
  public ToggleClaw(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
    if (clawSubsystem.isClawClosed()) {
      clawSubsystem.open();
    } else {
      clawSubsystem.close();
    }
  }
}
