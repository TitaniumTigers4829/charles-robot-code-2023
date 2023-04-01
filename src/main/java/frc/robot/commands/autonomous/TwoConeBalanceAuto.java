package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.PickupGamePiece;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.SetArmRotation;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TwoConeBalanceAuto extends SequentialCommandGroup {

  public TwoConeBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      // 1. Places a cube
      // new PlaceGamePiece(armSubsystem, clawSubsystem, ArmConstants.PLACE_HIGH_ROTATION, ArmConstants.PLACE_HIGH_EXTENSION).withTimeout(2),

      // 2. Drives over and picks up a cone
      // new ParallelCommandGroup(
      //   new InstantCommand(clawSubsystem::setCargoModeCone),
      //   new SetArmExtension(armSubsystem, 0.01),
      //   new ParallelRaceGroup(
      //     new PickupGamePiece(armSubsystem, clawSubsystem, ArmConstants.PICKUP_GROUND_ROTATION, ArmConstants.PICKUP_GROUND_EXTENSION),
      //     new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_FIRST, true)
      //   )
      // )

      // // 3. Drives back to the nodes while stowing the arm and then places the cone
      // new ParallelRaceGroup(
      //   new SequentialCommandGroup(
      //     new SetArmExtension(armSubsystem, 0.01),
      //     // new SetArmRotation(armSubsystem, ArmConstants.PLACE_MIDDLE_AUTO_ROTATION + 5),
      //     new PlaceGamePiece(armSubsystem, clawSubsystem, ArmConstants.PLACE_MIDDLE_AUTO_ROTATION, ArmConstants.PLACE_MIDDLE_AUTO_EXTENSION)
      //   ),
      //   new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_SECOND)
      // )

      // new MoveArmToStowed(armSubsystem, clawSubsystem)
    );
  }
}
