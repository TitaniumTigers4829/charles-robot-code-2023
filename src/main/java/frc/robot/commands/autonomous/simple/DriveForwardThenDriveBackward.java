// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.simple;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveForwardThenDriveBackward extends SequentialCommandGroup {
  /** Creates a new DriveForwardThenDriveBackward. */
  public DriveForwardThenDriveBackward(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new DriveCommand(driveSubsystem, visionSubsystem, () -> 0.25, () -> 0, () -> 0, () -> false).withTimeout(1),
      // allow a little time for the robot to not die from instantly reversing the motors
      new WaitCommand(0.2),
      new DriveCommand(driveSubsystem, visionSubsystem, () -> -0.5, () -> 0, () -> 0, () -> false).withTimeout(3)
    );
  }
}
