// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class GetSwerveModuleAngle extends CommandBase {
  
  DriveSubsystem driveSubsystem;

  /** This is just a test command. It displays the raw rotation in degrees of the CANCoders on shuffleboard. */
  public GetSwerveModuleAngle(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    SwerveModulePosition[] swerveModulePositions = driveSubsystem.getModulePositions(); 
    SmartDashboard.putNumber("Front Left Angle:", swerveModulePositions[0].angle.getDegrees());
    SmartDashboard.putNumber("Rear Left Angle:", swerveModulePositions[1].angle.getDegrees());
    SmartDashboard.putNumber("Front Right Angle:", swerveModulePositions[2].angle.getDegrees());
    SmartDashboard.putNumber("Rear Right Angle:", swerveModulePositions[3].angle.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
