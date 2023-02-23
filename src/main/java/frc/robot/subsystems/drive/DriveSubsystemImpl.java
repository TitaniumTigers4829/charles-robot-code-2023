// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemImpl extends SubsystemBase implements DriveSubsystem {

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.frontLeftDriveMotorPort,
    DriveConstants.frontLeftTurningMotorPort,
    DriveConstants.frontLeftTurningEncoderPort,
    DriveConstants.frontLeftAngleZero,
    DriveConstants.frontLeftTurningEncoderReversed,
    DriveConstants.frontLeftDriveEncoderReversed
  );
  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.frontRightDriveMotorPort,
    DriveConstants.frontRightTurningMotorPort,
    DriveConstants.frontRightTurningEncoderPort,
    DriveConstants.frontRightAngleZero,
    DriveConstants.frontRightTurningEncoderReversed,
    DriveConstants.frontRightDriveEncoderReversed
  );
  private final SwerveModule rearLeft = new SwerveModule(
    DriveConstants.rearLeftDriveMotorPort,
    DriveConstants.rearLeftTurningMotorPort,
    DriveConstants.rearLeftTurningEncoderPort,
    DriveConstants.rearLeftAngleZero,
    DriveConstants.rearLeftTurningEncoderReversed,
    DriveConstants.rearLeftDriveEncoderReversed
  );
  private final SwerveModule rearRight = new SwerveModule(
      DriveConstants.rearRightDriveMotorPort,
      DriveConstants.rearRightTurningMotorPort,
      DriveConstants.rearRightTurningEncoderPort,
      DriveConstants.rearRightAngleZero,
      DriveConstants.rearRightTurningEncoderReversed,
      DriveConstants.rearRightDriveEncoderReversed
    );

  private final SwerveModule[] swerveModules = {
    frontLeft,
    frontRight,
    rearLeft,
    rearRight
  };

  private int gyroOffset = 0;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.driveKinematics,
      getRotation2d(), getModulePositions());

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystemImpl() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        getRotation2d(),
        getModulePositions()
    );

    SmartDashboard.putString("Odometry", odometry.getPoseMeters().toString());
  }

  @Override
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.joystickMaxSpeedMetersPerSecondLimit);
    
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  @Override
  public double getHeading() {
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  // /**
  //  * @return Heading in degrees from -180 to 180.
  //  */
  // public double getHeading() {
  //   return ((gyro.getAngle() + 180 + this.gyroOffset) % 360) - 180;
  // }

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void setGyroOffset(int gyroOffset) {
    this.gyroOffset = gyroOffset;    
  }

  @Override
  public void zeroHeading() {
    gyroOffset = 0;
    gyro.reset();
  }

  @Override
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    };

    return swerveModulePositions;
  }

  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.joystickMaxSpeedMetersPerSecondLimit);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(desiredStates[i]);
    }
  }

}
