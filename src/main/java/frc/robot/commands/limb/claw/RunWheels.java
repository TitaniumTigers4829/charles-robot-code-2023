// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limb.claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limb.ClawSubsystemImpl;

public class RunWheels extends CommandBase {

  private final ClawSubsystemImpl clawSubsystemImpl;


  public RunWheels(ClawSubsystemImpl clawSubsystemImpl) {
    this.clawSubsystemImpl = clawSubsystemImpl;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawSubsystemImpl.setMotorSpeed(0-9); //Change to Zevs Neo code (in Subsystem)
   
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}