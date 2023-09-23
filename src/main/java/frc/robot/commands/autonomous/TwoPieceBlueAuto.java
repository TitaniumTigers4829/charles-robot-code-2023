package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.ShootCubeAuto;
import frc.robot.commands.arm.auto.DropConeAuto;
import frc.robot.commands.arm.auto.MoveArmToPlaceConeAuto;
import frc.robot.commands.arm.auto.PickupConeAuto;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TwoPieceBlueAuto extends SequentialCommandGroup {
  
  public TwoPieceBlueAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, LEDSubsystem leds) {
        
    addCommands(
      new ShootCubeAuto(armSubsystem, clawSubsystem),
      
      new ParallelCommandGroup(
        new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, leds, TrajectoryConstants.THREE_PIECE_BALANCE_AUTO_FIRST_BLUE, true, 4.5, 3.5),
        new PickupConeAuto(armSubsystem, clawSubsystem)
      ),

      new ParallelCommandGroup(
        new FollowPathPlannerTrajectory(driveSubsystem, visionSubsystem, leds, TrajectoryConstants.THREE_PIECE_BALANCE_AUTO_SECOND_BLUE),
        new MoveArmToPlaceConeAuto(armSubsystem, clawSubsystem)
      ),

      new DropConeAuto(clawSubsystem),
      new WaitCommand(.3),
      new MoveArmToStowed(armSubsystem, clawSubsystem)
    );
  }
}
