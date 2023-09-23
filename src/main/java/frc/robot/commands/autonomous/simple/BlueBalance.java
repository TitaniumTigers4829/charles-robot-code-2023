// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.simple;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.commands.autonomous.FollowPathPlannerTrajectory;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueBalance extends SequentialCommandGroup {
  /** Creates a new BlueBalance. */
  public BlueBalance(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, LEDSubsystem leds, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> NodeAndModeRegistry.setIsConeMode(true)),
      new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(19)),

      new PlaceGamePiece(armSubsystem, clawSubsystem, ()->false).withTimeout(3),
      new WaitCommand(.5),
      new MoveArmToStowed(armSubsystem, clawSubsystem),

      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, leds, "high cone, balance blue 1", true, 1, 1),

      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->0.1, ()->false).withTimeout(0.1)
    );
  }
}
