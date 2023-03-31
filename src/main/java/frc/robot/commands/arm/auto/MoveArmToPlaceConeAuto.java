package frc.robot.commands.arm.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.SetArmRotation;
import frc.robot.commands.claw.SetClawIntakeSpeed;
import frc.robot.commands.claw.SetClawRotation;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class MoveArmToPlaceConeAuto extends SequentialCommandGroup {

  public MoveArmToPlaceConeAuto(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      new SetClawIntakeSpeed(clawSubsystem, 0),
      new SetArmExtension(armSubsystem, ArmConstants.STOWED_EXTENSION),
      new SetClawRotation(clawSubsystem, 0),
      new SetArmRotation(armSubsystem, ArmConstants.PLACE_HIGH_ROTATION),
      new SetArmExtension(armSubsystem, ArmConstants.PLACE_HIGH_EXTENSION)
    );
  }
  
}
