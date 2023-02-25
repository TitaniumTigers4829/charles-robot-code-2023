// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.sql.Time;
import java.util.function.DoubleSupplier;

public class Balance extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier rightX;
  private final boolean fromLeft;

  private double oldTime;
  private double pitchRateOfChange;
  private double oldPitch;
  private boolean firstLatch;
  private boolean secondLatch;

  private PIDController balancePidController;



  /** Creates a new Balance.
   * @param fromLeft true if approaching from the left side, false if approaching from the right.
   */
  public Balance(DriveSubsystem driveSubsystem, DoubleSupplier rightX, boolean fromLeft) {
    this.rightX = rightX;
    this.fromLeft = fromLeft;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    firstLatch = false;
    secondLatch = false;
    balancePidController = new PIDController(
            BalanceConstants.pBalance,
            BalanceConstants.iBalance,
            BalanceConstants.dBalance
    );
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Balancing", true);
    double pitch = driveSubsystem.gyro.getPitch();
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Old Pitch", oldPitch);

    // Calculates the change in pitch per second.
    pitchRateOfChange = (oldPitch - pitch) / (oldTime - System.currentTimeMillis()) * 1000f;
    oldPitch = pitch;
    oldTime = System.currentTimeMillis();
    SmartDashboard.putNumber("dPitch/dTime", pitchRateOfChange);


    if (Math.abs(pitch) > BalanceConstants.initializationPitch) {
      firstLatch = true;
    }

    if (Math.abs(pitch) < BalanceConstants.minPitchDegrees) {
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
      initialDrive(balancePidController.calculate(pitch, 0));
    } else {
      initialDrive(BalanceConstants.initialSpeed);
    }

  
  }

  private void initialDrive(double speed) {

    double driveSpeed = speed;
    if (fromLeft) {
      driveSpeed *= -1;
    }

    driveSubsystem.drive(
            driveSpeed,
            0,
            0,
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
