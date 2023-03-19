// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class PlaceGamePiece extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private double rotation;
  private double extension;
  private Timer timer;

  public PlaceGamePiece(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, double rotation, double extension) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    addRequirements(this.armSubsystem);
    this.rotation = rotation;
    this.extension = extension;
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    armSubsystem.resetRotationController();
    armSubsystem.unlockExtensionSolenoid();
    if (armSubsystem.getCargoMode() == "Cone") {
      clawSubsystem.setWristPosition(0);
      rotation -= 3;
      extension += 0.03;
      clawSubsystem.setIntakeSpeed(-0.075);
    } else {
      clawSubsystem.setWristPosition(180);
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
    clawSubsystem.open();
    if (armSubsystem.getCargoMode() == "Cube") {
      clawSubsystem.setIntakeSpeed(-.08);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
