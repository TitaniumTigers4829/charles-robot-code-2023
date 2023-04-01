package frc.robot.commands.deprecated;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PlaceGamePieceOld extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;

  private final double rotation;
  private final double extension;

  public PlaceGamePieceOld(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, double rotation, double extension) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.armSubsystem, this.clawSubsystem);
    this.rotation = rotation;
    this.extension = extension;
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    if (clawSubsystem.isConeMode()) {
      if (rotation > 180) {
        clawSubsystem.setWristPosition(0);
      } else {
        clawSubsystem.setWristPosition(180);
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
    clawSubsystem.open();
    if (!clawSubsystem.isConeMode()) {
      clawSubsystem.setIntakeSpeed(ClawConstants.PLACE_CUBE_INTAKE_SPEED);
    } else {
      clawSubsystem.setIntakeSpeed(ClawConstants.PLACE_CONE_INTAKE_SPEED);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
