package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.PickupGamePiece;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TwoConeBalanceAuto extends SequentialCommandGroup {

  public TwoConeBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      // 1. Places the first cone
      // new PlaceGamePiece(armSubsystem, clawSubsystem, ArmConstants.PLACE_HIGH_ROTATION, ArmConstants.PLACE_HIGH_EXTENSION).withTimeout(2),

      // 2. Drives over to the second cone
      new ParallelCommandGroup(
        // new MoveArmToStowed(armSubsystem, clawSubsystem).andThen(new WaitCommand(3)).andThen(new PickupGamePiece(armSubsystem, clawSubsystem, ArmConstants.PICKUP_GROUND_ROTATION, ArmConstants.PICKUP_GROUND_EXTENSION).withTimeout(5)),
        new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_FIRST, true)
      ).withTimeout(9),

      // 4. Drives back to the nodes
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_SECOND).withTimeout(9.94),

      // 5. Places the second cone
      new PlaceGamePiece(armSubsystem, clawSubsystem, ArmConstants.PLACE_HIGH_ROTATION, ArmConstants.PLACE_HIGH_EXTENSION).withTimeout(2)
    );
  }
}
