package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PlaceGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;

  private double rotation;
  private double extension;
  private BooleanSupplier incrementHeight;

  public PlaceGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, BooleanSupplier incrementHeight) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.incrementHeight = incrementHeight;
    addRequirements(armSubsystem, clawSubsystem);
  }

  @Override
  public void initialize() {
    rotation = (NodeAndModeRegistry.getSelectedNode() > 18 ? ArmConstants.PLACE_HIGH_ROTATION : (NodeAndModeRegistry.getSelectedNode() > 9 ? ArmConstants.PLACE_MIDDLE_ROTATION : ArmConstants.PLACE_LOW_ROTATION));
    extension = (NodeAndModeRegistry.getSelectedNode() > 18 ? ArmConstants.PLACE_HIGH_EXTENSION : (NodeAndModeRegistry.getSelectedNode() > 9 ? ArmConstants.PLACE_MIDDLE_EXTENSION : ArmConstants.PLACE_LOW_EXTENSION));

    armSubsystem.resetExtensionController();
    if (NodeAndModeRegistry.isConeMode()) {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(ClawConstants.HOLD_CONE_INTAKE_SPEED);
      if (rotation > 180) {
        clawSubsystem.setWristPosition(0);
      } else {
        clawSubsystem.setWristPosition(180);
      }
    } else {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(ClawConstants.HOLD_CUBE_INTAKE_SPEED);
    }
  }

  @Override
  public void execute() {
    if (incrementHeight.getAsBoolean()) {
      armSubsystem.setRotation(rotation - 1.5);
      if (Math.abs(rotation - 1.5 - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
        armSubsystem.setExtension(extension);
      }
    } else if (!incrementHeight.getAsBoolean()) {
      if (NodeAndModeRegistry.isConeMode()) {
        armSubsystem.setRotation(rotation);
        if (Math.abs(rotation - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
          armSubsystem.setExtension(extension);
        }
      } else {
        armSubsystem.setRotation(rotation + 1.5);
        if (Math.abs(rotation + 1.5 - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
          armSubsystem.setExtension(extension);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.open();
    if (!NodeAndModeRegistry.isConeMode()) {
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
