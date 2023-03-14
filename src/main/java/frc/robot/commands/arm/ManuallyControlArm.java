// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ManuallyControlArm extends CommandBase {

  private final ArmSubsystem armSubsystem;

  private final DoubleSupplier rotationSpeed;
  private final DoubleSupplier extensionSpeed;
  
  public ManuallyControlArm(ArmSubsystem armSubsystem, DoubleSupplier rotationSpeed, DoubleSupplier extensionSpeed) {
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);
    this.rotationSpeed = rotationSpeed;
    this.extensionSpeed = extensionSpeed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // armSubsystem.setRotationSpeed(rotationSpeed.getAsDouble() / 2.0);
    // if (!armSubsystem.isExtensionMotorStalling() && Math.abs(extensionSpeed.getAsDouble()) > 0.1) {
    //   armSubsystem.unlockExtensionSolenoid();
    //   armSubsystem.setCurrentExtensionSpeed(extensionSpeed.getAsDouble() / 2.0);
    // } else {
    //   armSubsystem.lockExtensionSolenoid();
    //   armSubsystem.setCurrentExtensionSpeed(-0.075);
    // }

    if (Math.abs(rotationSpeed.getAsDouble()) > 0.1) {
      armSubsystem.setRotationSpeed(rotationSpeed.getAsDouble() / 2.0);
    } else {
      armSubsystem.setRotationSpeed(0);
      // armSubsystem.stopArmMotorFromMoving();
    }
    // if (Math.abs(extensionSpeed.getAsDouble()) > 0.1 && armSubsystem.getCurrentExtension() < ArmConstants.MAX_EXTENSION_PROPORTION) {
    if (Math.abs(extensionSpeed.getAsDouble()) > 0.1) {
      armSubsystem.unlockExtensionSolenoid();
      armSubsystem.setCurrentExtensionSpeed(extensionSpeed.getAsDouble() / 2.0);
    } else {
      armSubsystem.lockExtensionSolenoid();
      armSubsystem.setCurrentExtensionSpeed(0.075);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
