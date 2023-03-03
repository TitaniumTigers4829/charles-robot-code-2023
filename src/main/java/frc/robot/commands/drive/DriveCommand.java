// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveCommand extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier leftY, leftX, rightX;
  private final BooleanSupplier isFieldRelative;


  /**
   * The command for driving the robot using joystick inputs.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftY The joystick input for driving forward and backwards
   * @param leftX The joystick input for driving left and right
   * @param rightX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive
   * field relative
   */
  public DriveCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier isFieldRelative) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
    this.leftY = leftY;
    this.leftX = leftX;
    this.rightX = rightX;
    this.isFieldRelative = isFieldRelative;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Drives the robot
    driveSubsystem.drive(
      leftY.getAsDouble() * DriveConstants.joystickMaxSpeedMetersPerSecondLimit,
      leftX.getAsDouble() * DriveConstants.joystickMaxSpeedMetersPerSecondLimit,
      rightX.getAsDouble() * DriveConstants.maxAngularSpeedRadiansPerSecond,
      isFieldRelative.getAsBoolean()
    );

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}