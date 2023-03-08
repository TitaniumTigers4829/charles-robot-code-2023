// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final boolean fromLeft;

  private boolean firstLatch;
  private boolean secondLatch;

  private final PIDController balancePidController = new PIDController(
    BalanceConstants.BALANCE_P,
    BalanceConstants.BALANCE_I,
    BalanceConstants.BALANCE_D
  );
  private final ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.FACEFORWARD_P, 0, 0, TrajectoryConstants.THETA_CONTROLLER_CONSTRAINTS);


  /** Creates a new Balance.
   * @param fromLeft true if approaching from the left side, false if approaching from the right.
   */
  public Balance(DriveSubsystem driveSubsystem, boolean fromLeft) {
    this.fromLeft = fromLeft;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    firstLatch = false;
    secondLatch = false; 
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Balancing", true);
    double error = driveSubsystem.getBalanceError();
    SmartDashboard.putNumber("Pitch",  driveSubsystem.getPitch());
    SmartDashboard.putNumber("Roll",  driveSubsystem.getRoll());
    SmartDashboard.putNumber("Yaw",  driveSubsystem.getHeading());
    SmartDashboard.putNumber("Balance Error",  driveSubsystem.getBalanceError());


    if (Math.abs(error) > BalanceConstants.BALANCE_ERROR_INIT_DEGREES) {
      firstLatch = true;
    }

    if (Math.abs(error) < BalanceConstants.BALANCE_ERROR_NEAR_BALANCED) {
      // Has surpassed the limits.
      if (firstLatch) {
        secondLatch = true;
        SmartDashboard.putBoolean("Triggering", true);
      } else {
        SmartDashboard.putBoolean("Triggering", false);

      }

    }
    SmartDashboard.putBoolean("Second Latch", secondLatch);

    SmartDashboard.putBoolean("First Latch", firstLatch);

    if (secondLatch) {
      if (Math.abs(error) < BalanceConstants.BALANCE_ERROR_CONSIDERED_BALANCED) {
        initialDrive(0, true);
      } else {
        initialDrive(-1 * balancePidController.calculate(error, 0), true);
      }
    } else {
      initialDrive(BalanceConstants.INITIAL_SPEED, false);
    }

  
  }

  private void initialDrive(double speed, boolean faceForward) {

    double driveSpeed = speed;
    if (fromLeft) {
      driveSpeed *= -1;
    }

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

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
