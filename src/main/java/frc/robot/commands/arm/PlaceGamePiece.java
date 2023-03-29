package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PlaceGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;

  private final double rotation;
  private final double extension;

  public PlaceGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, double rotation, double extension) {
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
      clawSubsystem.setWristPosition(0);
      clawSubsystem.setIntakeSpeed(-0.075);
    }
    SmartDashboard.putBoolean("running", true);
  }

  @Override
  public void execute() {
    armSubsystem.setRotation(rotation);
    if (Math.abs(rotation - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
      armSubsystem.setExtension(extension);
    }
    SmartDashboard.putBoolean("running", true);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    clawSubsystem.open();
    if (!clawSubsystem.isConeMode()) {
      clawSubsystem.setIntakeSpeed(-.08);
    }
    SmartDashboard.putBoolean("running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
