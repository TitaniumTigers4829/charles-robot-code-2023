// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PickupGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  // private final double rotation = 105.3;
  private final double rotation = 110.7;
  // private final double extension = 1.39;
  private final double extension = .768;
  private Timer timer;

  public PickupGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    armSubsystem.unlockExtensionSolenoid();
    clawSubsystem.setWristPosition(180);
    if (!clawSubsystem.isConeMode()) {
      clawSubsystem.open();
      clawSubsystem.setIntakeSpeed(.15);
    } else {
      clawSubsystem.close();
      clawSubsystem.setIntakeSpeed(.25);
    }
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    armSubsystem.setRotation(rotation);
    if (timer.get() > 0.1) { // 10 ticks
      armSubsystem.setExtension(extension);
    } else {
      armSubsystem.setExtensionSpeed(ArmConstants.ARM_MOVE_SPEED_BEFORE_REAL_MOVE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    if (clawSubsystem.isConeMode()) {
      clawSubsystem.setIntakeSpeed(.08);
    } else {
      clawSubsystem.setIntakeSpeed(0);
    }
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
