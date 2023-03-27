package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PickupGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;

  private final double rotation;
  private final double extension;

  public PickupGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, double rotation, double extension) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.armSubsystem);
    this.rotation = rotation;
    this.extension = extension;
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    clawSubsystem.setWristPosition(180);
    if (!clawSubsystem.isConeMode()) {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(.15);
    } else {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(.25);
    }
  }

  @Override
  public void execute() {
    armSubsystem.setRotation(rotation);
    armSubsystem.setExtension(extension);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    if (clawSubsystem.isConeMode()) {
      clawSubsystem.setIntakeSpeed(.08);
    } else {
      clawSubsystem.setIntakeSpeed(0);
    }
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
