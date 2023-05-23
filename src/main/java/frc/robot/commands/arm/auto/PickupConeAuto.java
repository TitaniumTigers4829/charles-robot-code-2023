package frc.robot.commands.arm.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.SetArmRotation;
import frc.robot.commands.claw.SetClawIntakeSpeed;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PickupConeAuto extends SequentialCommandGroup {

  public PickupConeAuto(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      new InstantCommand(()->NodeAndModeRegistry.setIsConeMode(true)),
      new SetClawIntakeSpeed(clawSubsystem, 0),
      new SetArmExtension(armSubsystem, ArmConstants.STOWED_EXTENSION),
      new SetArmRotation(armSubsystem, ArmConstants.PICKUP_GROUND_ROTATION_AUTO),
      new SetClawIntakeSpeed(clawSubsystem, ClawConstants.PICKUP_CONE_INTAKE_SPEED),
      new SetArmExtension(armSubsystem, ArmConstants.PICKUP_GROUND_EXTENSION_AUTO)
    );
  }
}
