package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
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
    addRequirements(this.armSubsystem, this.clawSubsystem);
    this.rotation = rotation;
    this.extension = extension;
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    if (!clawSubsystem.isConeMode()) {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(.15);
      clawSubsystem.setWristPosition(0);
    } else {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(.5);
      if (rotation < 180) {
        clawSubsystem.setWristPosition(180);
      } else {
        clawSubsystem.setWristPosition(0);
      }
    }
  }

  @Override
  public void execute() {
    armSubsystem.setRotation(rotation);
    if (Math.abs(rotation - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
      armSubsystem.setExtension(extension);
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Done", "pickup done");
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    if (clawSubsystem.isConeMode()) {
      clawSubsystem.setIntakeSpeed(0);
    } else {
      clawSubsystem.setIntakeSpeed(0.04);
    }
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
