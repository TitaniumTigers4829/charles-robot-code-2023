// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ManualClaw extends CommandBase {

  private ClawSubsystem clawSubsystem;
  private ArmSubsystem armSubsystem;
  private BooleanSupplier intakeCargo;
  private BooleanSupplier expelCargo;
  private BooleanSupplier rotateClaw180;
  private DoubleSupplier armRotationSpeed;
  private DoubleSupplier armExtensionSpeed;
  private double initialRotation;
  private boolean lastState;
  private boolean lastStateOff;
  private int ticksFromName;

  public ManualClaw(ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, BooleanSupplier intakeCargo, BooleanSupplier expelCargo, DoubleSupplier armRotationSpeed, DoubleSupplier armExtensionSpeed, BooleanSupplier rotateClaw) {
    this.clawSubsystem = clawSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeCargo = intakeCargo;
    this.expelCargo = expelCargo;
    this.rotateClaw180 = rotateClaw;
    this.armRotationSpeed = armRotationSpeed;
    this.armExtensionSpeed = armExtensionSpeed;
    addRequirements(clawSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    initialRotation = clawSubsystem.getWristAngle();
    lastState = false;
    ticksFromName = 0;
    lastStateOff = false;
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
        clawSubsystem.setIntakeSpeed(0.06);
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

    if (Math.abs(armRotationSpeed.getAsDouble()) > 0.1) {
      armSubsystem.setRotationSpeed(armRotationSpeed.getAsDouble() / 1);
    } else {
      armSubsystem.setRotationSpeed(0);
    }

    if (Math.abs(armExtensionSpeed.getAsDouble()) > 0.1) {
      ticksFromName++;
      if (lastStateOff) {
        armSubsystem.unlockExtensionSolenoid();
        lastStateOff = false;
      } else if (ticksFromName > 10) {
        armSubsystem.setExtensionSpeed(armExtensionSpeed.getAsDouble());
      } else {
        armSubsystem.setExtensionSpeed(ArmConstants.ARM_MOVE_SPEED_BEFORE_REAL_MOVE);
      }

    } else {
      lastStateOff = true;
      ticksFromName = 0;
      armSubsystem.lockExtensionSolenoid();
      armSubsystem.setExtensionSpeed(0.075);
    }

    if (lastState != rotateClaw180.getAsBoolean()) {
        lastState = rotateClaw180.getAsBoolean();
        initialRotation = clawSubsystem.getWristAngle();
    }

    if (rotateClaw180.getAsBoolean()) {
      if (initialRotation > 90) {
        clawSubsystem.setWristPosition(0);
      } else {
        clawSubsystem.setWristPosition(180);
        }
      } else {
        clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
      }
  }


  @Override
  public void end(boolean interrupted) {
    armSubsystem.setExtensionSpeed(0);
    armSubsystem.setRotationSpeed(0);
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
    clawSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
