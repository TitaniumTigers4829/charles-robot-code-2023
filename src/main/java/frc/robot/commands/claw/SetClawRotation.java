package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawRotation extends InstantCommand {
  
  private final ClawSubsystem clawSubsystem;
  private final double rotation;

  public SetClawRotation(ClawSubsystem clawSubsystem, double rotation) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    clawSubsystem.setWristPosition(rotation);
  }
}
