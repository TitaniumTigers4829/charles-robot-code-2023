// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmRotation extends CommandBase {
  private ArmSubsystem armSubsystem;
  private double rotationDegrees;
  /** Creates a new SetArmRotation. */
  public SetArmRotation(ArmSubsystem armSubsystem, double rotationDegrees) {
    this.armSubsystem = armSubsystem;
    this.rotationDegrees = rotationDegrees;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.goToAngle(rotationDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armSubsystem.getAngle() - rotationDegrees) < ArmConstants.ROTATION_ACCEPTABLE_ERROR;
  }
}
