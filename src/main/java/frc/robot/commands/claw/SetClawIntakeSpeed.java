package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawIntakeSpeed extends InstantCommand {

  private final ClawSubsystem clawSubsystem;
  private final double intakeSpeed;

  public SetClawIntakeSpeed(ClawSubsystem clawSubsystem, double intakeSpeed) {
    this.clawSubsystem = clawSubsystem;
    addRequirements(clawSubsystem);
    this.intakeSpeed = intakeSpeed;
  }

  @Override
  public void initialize() {
    clawSubsystem.setIntakeSpeed(intakeSpeed);
  }
}
