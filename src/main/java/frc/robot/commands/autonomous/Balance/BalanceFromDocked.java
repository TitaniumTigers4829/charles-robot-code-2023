// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class BalanceFromDocked extends CommandBase {

  private final DriveSubsystem driveSubsystem;

  private final PIDController balancePidController = new PIDController(
    BalanceConstants.BALANCE_P,
    BalanceConstants.BALANCE_I,
    BalanceConstants.BALANCE_D
  );

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    DriveConstants.FACEFORWARD_P, 
    0, 
    0, 
    TrajectoryConstants.THETA_CONTROLLER_CONSTRAINTS
  );

  /** 
   * Creates a new BalanceFromDocked.
   */
  public BalanceFromDocked(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  
    double error = driveSubsystem.getBalanceError();

    // SmartDashboard.putNumber("Total Balance Error", error);

    if (Math.abs(error) < BalanceConstants.BALANCE_ERROR_CONSIDERED_BALANCED) {
        driveForward(0, true);
      } else {
        driveForward(-1 * balancePidController.calculate(error, 0), false);
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Drives the robot forward at a given rate.
   * @param faceForward whether or not to rotate the robot to orient itself forward.
   */
  private void driveForward(double speed, boolean faceForward) {
    double driveSpeed = speed;
    double rot = 0;

    if (faceForward && !(Math.abs(driveSubsystem.getHeading()) < BalanceConstants.ORIENTATION_ERROR_CONSIDERED_ORIENTED)) {
      rot = thetaController.calculate(driveSubsystem.getHeading(), 0);
    }

    driveSubsystem.drive(
      driveSpeed,
      0,
      rot,
      true
    );
  }

}