// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmRotation extends CommandBase {

  private ArmSubsystem armSubsystem;
  private double rotation;

  public SetArmRotation(ArmSubsystem armSubsystem, double rotation) {
    this.armSubsystem = armSubsystem;
    this.rotation = rotation;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.setRotation(rotation);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
