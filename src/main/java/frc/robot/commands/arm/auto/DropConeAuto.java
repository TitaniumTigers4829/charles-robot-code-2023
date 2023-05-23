package frc.robot.commands.arm.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.claw.SetClawIntakeSpeed;
import frc.robot.subsystems.claw.ClawSubsystem;

public class DropConeAuto extends SequentialCommandGroup {

  public DropConeAuto(ClawSubsystem clawSubsystem) {
    addCommands(
      new SetClawIntakeSpeed(clawSubsystem, ClawConstants.PLACE_CONE_INTAKE_SPEED),
      new OpenClaw(clawSubsystem)
    );
  }
  
}
