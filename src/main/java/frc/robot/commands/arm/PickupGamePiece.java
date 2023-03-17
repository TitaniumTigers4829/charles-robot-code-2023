// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PickupGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final double rotation = 108.5;
  private final double extension = .95;

  public PickupGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    armSubsystem.resetRotationController();
    armSubsystem.unlockExtensionSolenoid();
    clawSubsystem.setWristPosition(0);
    if (armSubsystem.getCargoMode() == "Cube") {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(.15);
    } else {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(.15);
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
    if (armSubsystem.getCargoMode() == "Cube") {
      clawSubsystem.setIntakeSpeed(.08);
    } else {
      clawSubsystem.setIntakeSpeed(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
