package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ToggleControlMode extends InstantCommand {
  
  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  
  public ToggleControlMode(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(armSubsystem, clawSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toggleControlMode();
    clawSubsystem.toggleControlMode();
  }
}
