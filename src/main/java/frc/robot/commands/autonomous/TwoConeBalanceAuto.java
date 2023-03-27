package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TwoConeBalanceAuto extends SequentialCommandGroup {

  public TwoConeBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      // 1. Places the first cone
      new PlaceGamePiece(armSubsystem, clawSubsystem, ArmConstants.PLACE_HIGH_ROTATION, ArmConstants.PLACE_HIGH_EXTENSION),

      // 2. Drives over to the second cone
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_FIRST, true),

      // 3. Picks up the second cone
      // new PickupGamePieceFromGround(armSubsystem, clawSubsystem),

      // 4. Drives back to the nodes
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_SECOND),

      // 5. Places the second cone
      new PlaceGamePiece(armSubsystem, clawSubsystem, ArmConstants.PLACE_HIGH_ROTATION, ArmConstants.PLACE_HIGH_EXTENSION),

      // 6. Drives onto the charging station
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_THIRD)
    );
  }
}
