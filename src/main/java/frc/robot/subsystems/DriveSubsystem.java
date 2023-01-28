// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class DriveSubsystem extends SubsystemBase {

  // The gyro sensor
  public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private int gyroOffset = 0;

  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.frontLeftDriveMotorPort,
      DriveConstants.frontLeftTurningMotorPort,
      DriveConstants.frontLeftTurningEncoderPort,
      DriveConstants.frontLeftAngleZero,
      DriveConstants.frontLeftTurningEncoderReversed,
      DriveConstants.frontLeftDriveEncoderReversed
    );
  private final SwerveModule rearLeft = new SwerveModule(
      DriveConstants.rearLeftDriveMotorPort,
      DriveConstants.rearLeftTurningMotorPort,
      DriveConstants.rearLeftTurningEncoderPort,
      DriveConstants.rearLeftAngleZero,
      DriveConstants.rearLeftTurningEncoderReversed,
      DriveConstants.rearLeftDriveEncoderReversed
    );
  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.frontRightDriveMotorPort,
      DriveConstants.frontRightTurningMotorPort,
      DriveConstants.frontRightTurningEncoderPort,
      DriveConstants.frontRightAngleZero,
      DriveConstants.frontRightTurningEncoderReversed,
      DriveConstants.frontRightDriveEncoderReversed
    );
  private final SwerveModule rearRight = new SwerveModule(
      DriveConstants.rearRightDriveMotorPort,
      DriveConstants.rearRightTurningMotorPort,
      DriveConstants.rearRightTurningEncoderPort,
      DriveConstants.rearRightAngleZero,
      DriveConstants.rearRightTurningEncoderReversed,
      DriveConstants.rearRightDriveEncoderReversed
    );

  // Odometry class for tracking robot pose
  public SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.driveKinematics,
      gyro.getRotation2d(), getModulePositions());

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d(),
        getModulePositions()
    );
  }

  public double getHeading() {
    return (gyro.getAngle() + this.gyroOffset) % 360;
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public void setGyroOffset(int gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose, keeps the current rotation.
   *
   * @param pose The Pose2d to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }
  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putBoolean("Field Relative:", fieldRelative);
    SwerveModuleState[] swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeft.getPosition(),
      rearLeft.getPosition(),
      frontRight.getPosition(),
      rearRight.getPosition()
    };

    return swerveModulePositions;
  } 

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyroOffset = 0;
    gyro.reset();
  }
  
}
