// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.simple;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.commands.claw.SetClawRotation;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PlaceConeHighAuto extends SequentialCommandGroup {
  /** Creates a new PlaceConeHighAuto. */
  public PlaceConeHighAuto(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->NodeAndModeRegistry.setIsConeMode(true)),
      new InstantCommand(()->NodeAndModeRegistry.setSelectedNode(19)),
      new InstantCommand(armSubsystem::resetExtensionController),
      new SetClawRotation(clawSubsystem, 0),
      new PlaceGamePiece(armSubsystem, clawSubsystem, ()->false).withTimeout(4),
      new WaitCommand(0.7),
      new MoveArmToStowed(armSubsystem, clawSubsystem)
    );
  }
}
