// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmExtension extends CommandBase {

  private ArmSubsystem armSubsystem;
  private double extension;

  public SetArmExtension(ArmSubsystem armSubsystem, double extension) {
    this.armSubsystem = armSubsystem;
    this.extension = extension;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
<<<<<<< HEAD
=======
    // armSubsystem.resetExtensionController();
>>>>>>> cce75229082836f45b66dfe2a6f372d1ac5dac9d
    armSubsystem.unlockExtensionSolenoid();
  }

  @Override
  public void execute() {
    armSubsystem.setExtension(extension);
  }

  @Override
  public void end(boolean interrupted) {
   armSubsystem.setExtensionSpeed(0.01);
   armSubsystem.lockExtensionSolenoid();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(extension - armSubsystem.getExtension()) < ArmConstants.EXTENSION_ACCEPTABLE_ERROR;
  }
}
