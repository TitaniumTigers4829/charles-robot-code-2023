// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToStowedAfterPlacing;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SimpleAuto extends SequentialCommandGroup {

  public SimpleAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {

    addCommands(
      // Zero wrist
      new InstantCommand(clawSubsystem::zeroWristEncoder),
      // Place Cone
      new PlaceGamePiece(armSubsystem, clawSubsystem, 241, 0.97).withTimeout(2),
      new MoveArmToStowedAfterPlacing(armSubsystem, clawSubsystem, () -> false).withTimeout(2),
      // Back up over charge station.
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0.25, ()->0, ()->0, ()->false).withTimeout(.5),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->.135, ()->false).withTimeout(2),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->-0.25, ()->0, ()->0, ()->false).withTimeout(3.6), // 4.5
      // Back up to balance and stop
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->.135, ()->false).withTimeout(2),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->-.25, ()->0, ()->0, ()->false).withTimeout(2.17), // 2.75
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->.1, ()->false).withTimeout(.2),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->0, ()->false).withTimeout(.2)
    );
  }
}