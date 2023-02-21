// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Balance extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier rightX;
  private final boolean fromLeft;

  private double oldTime;
  private double pitchRateOfChange;
  private double oldPitch;

  private boolean balancing;

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
  public void initialize() {}

  @Override
  public void execute() {
    double pitch = driveSubsystem.gyro.getPitch();

    // Calculates the change in pitch per second.
    pitchRateOfChange = (oldPitch - pitch) / (oldTime - System.currentTimeMillis());
    oldPitch = pitch;
    oldTime = System.currentTimeMillis();

    if (pitch < BalanceConstants.minPitchRadians || pitchRateOfChange > BalanceConstants.maxPitchRadiansPerSecond) {
      // Has surpassed the limits.
      balancing = true;
    }

    if (!balancing) {
      initialDrive();
    }


  }

  private void initialDrive() {

    double driveSpeed = BalanceConstants.initialSpeed;
    if (!fromLeft) {
      driveSpeed *= -1;
    }

    driveSubsystem.drive(
            0,
            driveSpeed,
            rightX.getAsDouble() * DriveConstants.maxAngularSpeedRadiansPerSecond,
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
