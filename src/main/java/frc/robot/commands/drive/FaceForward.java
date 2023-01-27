// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FaceForward extends CommandBase {

  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier leftY, leftX;
  private final BooleanSupplier isFieldRelative;
  
  private final ProfiledPIDController thetaController = new ProfiledPIDController(0.01, 0, 0, PathPlannerConstants.thetaControllerConstraints);

  /** Creates a new FaceForward. */
  public FaceForward(DriveSubsystem driveSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, BooleanSupplier isFieldRelative) {
    this.driveSubsystem = driveSubsystem;
    this.leftY = leftY;
    this.leftX = leftX;
    this.isFieldRelative = isFieldRelative;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(
      leftY.getAsDouble() * DriveConstants.maxSpeedMetersPerSecond,
      leftX.getAsDouble() * DriveConstants.maxSpeedMetersPerSecond,
      thetaController.calculate(driveSubsystem.getHeading(), 0), 
      isFieldRelative.getAsBoolean()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
