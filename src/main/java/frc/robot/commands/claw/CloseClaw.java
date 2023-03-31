package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class CloseClaw extends InstantCommand {

  private final ClawSubsystem clawSubsystem;

  public CloseClaw(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
    clawSubsystem.close();
  }
}
