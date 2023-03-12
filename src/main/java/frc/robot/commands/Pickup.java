// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.arm.ArmSubsystem;
// import frc.robot.subsystems.claw.ClawSubsystem;

// public class Pickup extends CommandBase {

//   private final ArmSubsystem armSubsystem;
//   private final ClawSubsystem clawSubsystem;

//   public Pickup(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
//     this.armSubsystem = armSubsystem;
//     this.clawSubsystem = clawSubsystem;
//     addRequirements(armSubsystem, clawSubsystem);
//   }

//   @Override
//   public void initialize() {
//     armSubsystem.unlockExtensionSolenoid();
//     armSubsystem.setCurrentExtensionSpeed(.25);
//   }

//   @Override
//   public void execute() {}

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
