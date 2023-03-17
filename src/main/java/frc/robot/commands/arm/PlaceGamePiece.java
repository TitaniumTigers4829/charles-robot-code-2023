// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    addRequirements(this.armSubsystem);
    this.rotation = rotation;
    this.extension = extension;
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    armSubsystem.resetRotationController();
    armSubsystem.unlockExtensionSolenoid();
    if (armSubsystem.getCargoMode() == "Cone") {
      clawSubsystem.setWristPosition(180);
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
    clawSubsystem.open();
    if (armSubsystem.getCargoMode() == "Cube") {
      clawSubsystem.setIntakeSpeed(-.08);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
