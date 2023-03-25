// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PickupGamePieceFromGround extends CommandBase {
  private ArmSubsystem armSubsystem;
  private ClawSubsystem clawSubsystem;
  private boolean isCone;
  private Timer timer;
  /** Creates a new PickupGamePieceFromGround. */
  public PickupGamePieceFromGround(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, boolean isCone) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.isCone = isCone;
    addRequirements(armSubsystem, clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isCone) {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(0.25);
    } else {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(0.15);
    }
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 0.1) {
      armSubsystem.setExtensionSpeed(ArmConstants.ARM_MOVE_SPEED_BEFORE_REAL_MOVE);
    } else {
      armSubsystem.setExtension(ArmConstants.PICKUP_FROM_GROUND_EXTENSION);
    }
    armSubsystem.setRotation(ArmConstants.PICKUP_FROM_GROUND_ROTATION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    clawSubsystem.setIntakeSpeed(0.06);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
