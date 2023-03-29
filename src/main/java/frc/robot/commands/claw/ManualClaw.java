package frc.robot.commands.claw;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ManualClaw extends CommandBase {

  private ClawSubsystem clawSubsystem;
  private BooleanSupplier intakeCargo;
  private BooleanSupplier expelCargo;
  private BooleanSupplier rotateClaw180;
  private double initialRotation;
  private boolean lastState;
  private boolean hasSetPos;

  public ManualClaw(ClawSubsystem clawSubsystem, BooleanSupplier intakeCargo, BooleanSupplier expelCargo, BooleanSupplier rotateClaw) {
    this.clawSubsystem = clawSubsystem;
    this.intakeCargo = intakeCargo;
    this.expelCargo = expelCargo;
    this.rotateClaw180 = rotateClaw;
    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
    initialRotation = clawSubsystem.getWristAngle();
    lastState = false;
    hasSetPos = false;
  }

  @Override
  public void execute() {
    if (!clawSubsystem.isConeMode()) {
      clawSubsystem.open();
      if (intakeCargo.getAsBoolean()) {
        clawSubsystem.setIntakeSpeed(0.15);
      } else if (expelCargo.getAsBoolean()) {
        clawSubsystem.setIntakeSpeed(-0.15);
      } else {
        clawSubsystem.setIntakeSpeed(0.04);
      }
    } else {
      if (intakeCargo.getAsBoolean()) {
        clawSubsystem.close();
        clawSubsystem.setIntakeSpeed(0.25);
      } else if (expelCargo.getAsBoolean()) {
        clawSubsystem.setIntakeSpeed(0);
        clawSubsystem.open();
      } else {
        clawSubsystem.close();
        clawSubsystem.setIntakeSpeed(0);
      }
    }

    if (lastState != rotateClaw180.getAsBoolean()) {
        lastState = rotateClaw180.getAsBoolean();
        initialRotation = clawSubsystem.getWristAngle();
    }

    if (rotateClaw180.getAsBoolean()) {
      hasSetPos = false;
      if (initialRotation > 90) {
        clawSubsystem.setWristPosition(0);
      } else {
        clawSubsystem.setWristPosition(180);
        }
      } else {
        if (!hasSetPos) {
          clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
          hasSetPos = true;
        }
      }
  }


  @Override
  public void end(boolean interrupted) {
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
    clawSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
