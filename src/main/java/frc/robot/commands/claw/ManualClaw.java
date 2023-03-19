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
  private BooleanSupplier succCargo;
  private BooleanSupplier expelCargo;
  private BooleanSupplier rotateClaw180;
  private DoubleSupplier armRotationSpeed;
  private DoubleSupplier armExtensionSpeed;
  private double initialRotation;
  private boolean lastState;
  private boolean runOnce;
  private boolean lastStateOff;
  private int ticksFromName;
  /** Creates a new ManualClaw. */
  public ManualClaw(ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, BooleanSupplier succCargo, BooleanSupplier expelCargo, DoubleSupplier armRotationSpeed, DoubleSupplier armExtensionSpeed, BooleanSupplier rotateClaw) {
    this.clawSubsystem = clawSubsystem;
    this.armSubsystem = armSubsystem;
    this.succCargo = succCargo;
    this.expelCargo = expelCargo;
    this.rotateClaw180 = rotateClaw;
    this.armRotationSpeed = armRotationSpeed;
    this.armExtensionSpeed = armExtensionSpeed;
    addRequirements(clawSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialRotation = clawSubsystem.getWristAngle();
    lastState = false;
    runOnce = false;
    ticksFromName = 0;
    lastStateOff = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armSubsystem.getCargoMode().equals("Cube")) {
      clawSubsystem.open();
      if (succCargo.getAsBoolean()) {
        clawSubsystem.setIntakeSpeed(0.15);
      } else if (expelCargo.getAsBoolean()) {
        clawSubsystem.setIntakeSpeed(-0.15);
      } else {
        clawSubsystem.setIntakeSpeed(0.06);
      }
    } else {
      if (succCargo.getAsBoolean()) {
        clawSubsystem.close();
        clawSubsystem.setIntakeSpeed(0.25);
      } else if (expelCargo.getAsBoolean()) {
        // clawSubsystem.setIntakeSpeed(-0.15);
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
      // armSubsystem.stopArmMotorFromMoving();
    }
    // if (Math.abs(extensionSpeed.getAsDouble()) > 0.1 && armSubsystem.getCurrentExtension() < ArmConstants.MAX_EXTENSION_PROPORTION) {
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
    // if (rotateClaw180.getAsBoolean()) {
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


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setExtensionSpeed(0);
    armSubsystem.setRotationSpeed(0);
    // clawSubsystem.setWristMotorSpeed(0);
    clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
    clawSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
