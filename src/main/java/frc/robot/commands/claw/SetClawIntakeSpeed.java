package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawIntakeSpeed extends InstantCommand {

  private final ClawSubsystem clawSubsystem;

  public SetClawIntakeSpeed(ClawSubsystem clawSubsystem) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
    
  }
}
