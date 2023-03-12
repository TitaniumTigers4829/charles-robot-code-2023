// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmExtension extends CommandBase {

  private final PIDController extensionSpeedPidController = new PIDController(
    .001, 
    0, 
    0
  );

  private ArmSubsystem armSubsystem;
  private double extension;

  public SetArmExtension(ArmSubsystem armSubsystem, double extension) {
    this.armSubsystem = armSubsystem;
    this.extension = extension;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.unlockExtensionSolenoid();
  }

  @Override
  public void execute() {
    // Sets the extension speed to 100% in the direction it needs to go.
    if (Math.abs(extension - armSubsystem.getExtension()) > ArmConstants.EXTENSION_ACCEPTABLE_ERROR) {
      armSubsystem.setCurrentExtensionSpeed((armSubsystem.getExtension() < extension ? -.005 : -.5));
    } else {
      armSubsystem.setCurrentExtensionSpeed(-.075);
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setCurrentExtensionSpeed(0.1);
    armSubsystem.lockExtensionSolenoid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
