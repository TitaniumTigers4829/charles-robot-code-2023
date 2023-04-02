package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;

public class PickupGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final LEDSubsystem leds;
  private final boolean isLoadingStation;
  private double rotation;
  private double extension;

  private boolean increaseHeight = false;

  public PickupGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, LEDSubsystem leds, double rotation, double extension, boolean isLoadingStation) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.leds = leds;
    addRequirements(this.armSubsystem, this.clawSubsystem, this.leds);
    this.rotation = rotation;
    this.extension = extension;
    this.isLoadingStation = isLoadingStation;
  }

  @Override
  public void initialize() {
    increaseHeight = false;
    armSubsystem.resetExtensionController();
    if (!NodeAndModeRegistry.isConeMode()) {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(.15);
      clawSubsystem.setWristPosition(0);
      if (isLoadingStation) {
        increaseHeight = true;
      }
    } else {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(isLoadingStation ? 0.15 : .5);
    }

    if (rotation < 180) {
      clawSubsystem.setWristPosition(180);
    } else {
      clawSubsystem.setWristPosition(0);
    }
  }

  @Override
  public void execute() {
    if (!increaseHeight) {
      armSubsystem.setRotation(rotation);
      if (Math.abs(rotation - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
        armSubsystem.setExtension(extension);
      }
    } else {
      armSubsystem.setRotation(rotation + 1.5);
      if (Math.abs((rotation + 1.5) - armSubsystem.getRotation()) < ArmConstants.ROTATION_ACCEPTABLE_ERROR) {
        armSubsystem.setExtension(extension);
      }
    }

    if (!isLoadingStation && extension > 0.2) {
      double currentX = SmarterDashboardRegistry.getPose().getX();
      double chuteX = DriverStation.getAlliance() == Alliance.Blue ? DriveConstants.BLUE_CHUTE_X : DriveConstants.RED_CHUTE_X;
      double error = Math.abs(currentX - chuteX);
      if (error <= DriveConstants.LEDS_ACCEPTABLE_ERROR) {
        leds.setProcess(LEDProcess.GREEN);
      } else {
        leds.setProcess(LEDProcess.RED);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putString("Done", "pickup done");
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    if (NodeAndModeRegistry.isConeMode()) {
      clawSubsystem.setIntakeSpeed(0);
    } else {
      clawSubsystem.setIntakeSpeed(0.04);
    }
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
    if (NodeAndModeRegistry.isConeMode()) {
      leds.setProcess(LEDProcess.CONE);
    } else {
      leds.setProcess(LEDProcess.CUBE);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
