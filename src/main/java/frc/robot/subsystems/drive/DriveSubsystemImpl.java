// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemImpl extends SubsystemBase implements DriveSubsystem {

  private final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

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

  private final SwerveDrivePoseEstimator odometry;

  private int gyroOffset = 0;
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystemImpl() {
    odometry = new SwerveDrivePoseEstimator(
      DriveConstants.driveKinematics,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match
      stateStdDevs,
      visionMeasurementStdDevs
    );
  }

  @Override
  public void periodic() {
    // Uses the swerve's sensors to update the pose estimator
    odometry.update(
      getRotation2d(),
      getModulePositions()
    );
    
    SmartDashboard.putString("Estimated Pose", odometry.getEstimatedPosition().toString());
  }

  @SuppressWarnings("ParameterName")
  @Override
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getFieldRelativeRotation2d())
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

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public Rotation2d getFieldRelativeRotation2d() {
    double perspectiveOffset;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      perspectiveOffset = 0;
    } else {
      perspectiveOffset = 180;
    }
    return Rotation2d.fromDegrees(getHeading() + perspectiveOffset);
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
    return odometry.getEstimatedPosition();
  }
  
  @Override
  public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    // The gyro barely drifts throughout the match, so we trust it absolutely
    Pose2d visionMeasurementExcludingRotation = 
      new Pose2d(visionMeasurement.getX(), visionMeasurement.getY(), getRotation2d());
    odometry.addVisionMeasurement(visionMeasurementExcludingRotation, currentTimeStampSeconds);
  }

  @Override
  public void resetPoseEstimator(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  @Override
  public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }
  
  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.joystickMaxSpeedMetersPerSecondLimit); // TODO: Check if this has to be different
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(desiredStates[i]);
    }
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

}