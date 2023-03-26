// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class MoveArmToStowedAfterPickup extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private BooleanSupplier isFinishedSupplier;
  private final double rotationStage1 = 135;
  private final double rotationStage2 = 180;
  private boolean stage1Complete = false;
  private boolean stage2Complete = false;
  private final double extension = 0.01;
  private Timer timer;

  public MoveArmToStowedAfterPickup(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, BooleanSupplier isFinishedSupplier) {
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.isFinishedSupplier = isFinishedSupplier;
    addRequirements(this.armSubsystem, this.clawSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.resetExtensionController();
    armSubsystem.unlockExtensionSolenoid();
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.get() > 0.1) { // 10 ticks
      armSubsystem.setExtension(extension);
    } else {
      armSubsystem.setExtensionSpeed(ArmConstants.ARM_MOVE_SPEED_BEFORE_REAL_MOVE);
    }
    if (!stage1Complete) {
      armSubsystem.setRotation(rotationStage1);
      stage1Complete = (timer.get() > 0.3); // 30 ticks
    } else {
      armSubsystem.setRotation(rotationStage2);
      if (Math.abs(armSubsystem.getRotation() - rotationStage2) < ArmConstants.ROTATION_TOLERANCE_DEGREES) {
        stage2Complete = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    armSubsystem.lockExtensionSolenoid();
  }

  @Override
  public boolean isFinished() {
    return stage2Complete || isFinishedSupplier.getAsBoolean() || armSubsystem.getExtension() < 0.1;
    // return (Math.abs(rotation - armSubsystem.getRotation()) < 3.0
      // && Math.abs(extension - armSubsystem.getExtension()) < ArmConstants.EXTENSION_ACCEPTABLE_ERROR) || isFinishedSupplier.getAsBoolean();
  }
}
