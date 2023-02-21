// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final DoubleSupplier leftY, leftX, rightX;
  private final BooleanSupplier isFieldRelative;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier isFieldRelative) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.leftY = leftY;
    this.leftX = leftX;
    this.rightX = rightX;
    this.isFieldRelative = isFieldRelative;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(
      leftY.getAsDouble() * DriveConstants.joystickMaxSpeedMetersPerSecondLimit,
      leftX.getAsDouble() * DriveConstants.joystickMaxSpeedMetersPerSecondLimit,
      rightX.getAsDouble() * DriveConstants.maxAngularSpeedRadiansPerSecond,
      isFieldRelative.getAsBoolean()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
