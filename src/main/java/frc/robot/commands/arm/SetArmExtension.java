package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmExtension extends CommandBase {

  private ArmSubsystem armSubsystem;
  private double extension;

  public SetArmExtension(ArmSubsystem armSubsystem, double extension) {
    this.armSubsystem = armSubsystem;
    this.extension = extension;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
  }

  @Override
  public void execute() {
    armSubsystem.setExtension(extension);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(extension - armSubsystem.getExtension()) < ArmConstants.EXTENSION_ACCEPTABLE_ERROR;
  }
}
