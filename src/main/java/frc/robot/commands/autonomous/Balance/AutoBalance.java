package frc.robot.commands.autonomous.Balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemImpl;



public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(DriveSubsystem driveSubsystem, VisionSubsystemImpl visionSubsystem) {

        addCommands(
          new BalanceWithAprilTags(driveSubsystem, visionSubsystem, true),
          new BalanceFromDocked(driveSubsystem)
          );
    }
}
