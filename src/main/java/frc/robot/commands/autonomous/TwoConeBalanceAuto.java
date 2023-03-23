// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TwoConeBalanceAuto extends SequentialCommandGroup {

  /** Creates a new TwoConeBalanceAuto. */
  public TwoConeBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      // 1. Places the first cone
      // insert command here

      // 2. Drives over to the second cone
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_FIRST, true),

      // 3. Picks up the second cone
      // insert command here

      // 4. Drives back to the nodes
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_SECOND),

      // 5. Places the second cone
      // insert command here

      // 6. Drives in front of the charging station
      new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, TrajectoryConstants.TWO_CONE_BALANCE_AUTO_THIRD)

      // 7. Auto balances on the charging station
      // new Balance(driveSubsystem, true)
    );
  }
}
