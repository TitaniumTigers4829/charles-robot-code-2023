// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetRotationSpeed;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new SimpleAuto. */
  public SimpleAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetRotationSpeed(armSubsystem, -0.4),
      new WaitCommand(1.5),
      new SetRotationSpeed(armSubsystem, 0),
      new CloseClaw(clawSubsystem),
      new WaitCommand(0.5),
      new SetRotationSpeed(armSubsystem, 0.4),
      new WaitCommand(1.5),
      new SetRotationSpeed(armSubsystem, 0),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->-0.25, ()->0, ()->0, ()->false).withTimeout(4.5),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->.135, ()->false).withTimeout(2),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->-.25, ()->0, ()->0, ()->false).withTimeout(2.75),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->.1, ()->false).withTimeout(.2),
      new DriveCommand(driveSubsystem, visionSubsystem, ()->0, ()->0, ()->0, ()->false).withTimeout(.2)
    );
  }
}