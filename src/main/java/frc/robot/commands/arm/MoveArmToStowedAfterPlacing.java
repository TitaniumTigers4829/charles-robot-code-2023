// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class MoveArmToStowedAfterPlacing extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private BooleanSupplier isFinishedSupplier;
  private final double rotation = 180;
  private final double extension = 0.01;

  public MoveArmToStowedAfterPlacing(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, BooleanSupplier isFinishedSupplier) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.isFinishedSupplier = isFinishedSupplier;
    addRequirements(this.armSubsystem, this.clawSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.lockExtensionSolenoid();
  }
  

  @Override
  public void execute() {
    armSubsystem.setRotation(rotation);
    clawSubsystem.setWristPosition(180);
    if (Math.abs(extension - armSubsystem.getExtension()) < ArmConstants.EXTENSION_ACCEPTABLE_ERROR) {
      armSubsystem.setExtensionSpeed(0);
      armSubsystem.lockExtensionSolenoid();
    } else {
      armSubsystem.unlockExtensionSolenoid();
      armSubsystem.setExtension(extension);
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    armSubsystem.lockExtensionSolenoid();
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(rotation - armSubsystem.getRotation()) < 3.0
      && Math.abs(extension - armSubsystem.getExtension()) < ArmConstants.EXTENSION_ACCEPTABLE_ERROR) || isFinishedSupplier.getAsBoolean()
      || armSubsystem.getExtension() < 0.1;
  }
}
