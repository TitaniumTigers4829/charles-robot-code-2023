package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class OpenClaw extends InstantCommand {

  private final ClawSubsystem clawSubsystem;

  public OpenClaw(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
    clawSubsystem.open();
  }
}
