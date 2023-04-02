package frc.robot.commands.arm.teleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.SetArmRotation;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.claw.SetClawIntakeSpeed;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PlaceConeHigh extends SequentialCommandGroup {

  public PlaceConeHigh(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      new SetArmRotation(armSubsystem, ArmConstants.PLACE_HIGH_ROTATION),
      new SetArmExtension(armSubsystem, ArmConstants.PICKUP_GROUND_EXTENSION_AUTO),
      new OpenClaw(clawSubsystem),
      new SetClawIntakeSpeed(clawSubsystem, ClawConstants.PLACE_CONE_INTAKE_SPEED)
    );
  }
}
