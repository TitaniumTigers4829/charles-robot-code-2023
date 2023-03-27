package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class MoveArmToStowed extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final double armRotation = 180;
  private final double armExtension = 0.01;
  private final double wristRotation = 180;

  public MoveArmToStowed(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.armSubsystem, this.clawSubsystem);
  }

  @Override
  public void initialize() {}
  

  @Override
  public void execute() {
    armSubsystem.setRotation(armRotation);
    clawSubsystem.setWristPosition(wristRotation);
    if (Math.abs(armExtension - armSubsystem.getExtension()) < ArmConstants.EXTENSION_TOLERANCE) {
      armSubsystem.setExtensionSpeed(0);
      armSubsystem.lockExtensionSolenoid();
    } else {
      armSubsystem.unlockExtensionSolenoid();
      armSubsystem.setExtension(armExtension);
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(armSubsystem.getRotation() - armRotation) < ArmConstants.ROTATION_ACCEPTABLE_ERROR
      && Math.abs(armSubsystem.getExtension() - armExtension) < ArmConstants.EXTENSION_TOLERANCE);
  }
}
