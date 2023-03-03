// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limb.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.limb.ArmSubsystemImpl;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RotateArmToPose extends CommandBase {

  private final ArmSubsystemImpl armSubsystemImpl;


  public RotateArmToPose(ArmSubsystemImpl armSubsystemImpl) {
    this.armSubsystemImpl = armSubsystemImpl;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystemImpl.goToAngle(0-9);  

   
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}