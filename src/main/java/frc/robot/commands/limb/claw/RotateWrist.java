// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limb.claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.limb.ArmSubsystemImpl;
import frc.robot.subsystems.limb.ClawSubsystemImpl;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RotateWrist extends CommandBase {

  private final ClawSubsystemImpl clawSubsystemImpl;


  public RotateWrist(ClawSubsystemImpl clawSubsystemImpl) {
    this.clawSubsystemImpl = clawSubsystemImpl;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawSubsystemImpl.setWristAngle(180);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}