// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SimpleAuto extends SequentialCommandGroup {

  public SimpleAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {

    addCommands(
      new InstantCommand(() -> NodeAndModeRegistry.setIsConeMode(true)).withTimeout(0.1),
      new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(19)).withTimeout(0.1),
      new PlaceGamePiece(armSubsystem, clawSubsystem, ()->false).withTimeout(3),
      new WaitCommand(.7),
      new MoveArmToStowed(armSubsystem, clawSubsystem),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0.25, ()->0, ()->0, ()-> false).withTimeout(4.3)
    );
  }
}