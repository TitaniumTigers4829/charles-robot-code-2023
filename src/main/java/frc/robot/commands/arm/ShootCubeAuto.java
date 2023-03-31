package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.SetClawIntakeSpeed;
import frc.robot.commands.claw.SetClawRotation;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ShootCubeAuto extends SequentialCommandGroup {
  
  public ShootCubeAuto(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    addCommands(
      new InstantCommand(clawSubsystem::setCargoModeCube),
      new SetClawRotation(clawSubsystem, 0),
      new SetClawIntakeSpeed(clawSubsystem, ClawConstants.HOLD_CUBE_INTAKE_SPEED),
      new SetArmRotation(armSubsystem, ArmConstants.SHOOT_CUBE_MIDDLE_ROTATION),
      new SetClawIntakeSpeed(clawSubsystem, ClawConstants.SHOOT_CUBE_INTAKE_SPEED),
      new CloseClaw(clawSubsystem)
    );
  }
}
