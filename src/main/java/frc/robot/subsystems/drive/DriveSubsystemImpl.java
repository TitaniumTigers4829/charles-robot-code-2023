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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.dashboard.SmartDashboardLogger;

public class DriveSubsystemImpl extends SubsystemBase implements DriveSubsystem {

  // This will stay the same throughout the match. These values are harder to test for and tune, so assume this guess is right.
  private final Vector<N3> stateStandardDeviations = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  // This will be changed throughout the match depending on how confident we are that the limelight is right.
  private final Vector<N3> visionMeasurementStandardDeviations = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveModule frontLeftSwerveModule = new SwerveModule(
    DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
    DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
    DriveConstants.FRONT_LEFT_CANCODER_ID,
    DriveConstants.FRONT_LEFT_ZERO_ANGLE,
    DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
    DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
    "FL"
  );
  private final SwerveModule frontRightSwerveModule = new SwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
    DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
    DriveConstants.FRONT_RIGHT_CANCODER_ID,
    DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
    DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
    DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
    "FR"
  );
  private final SwerveModule rearLeftSwerveModule = new SwerveModule(
    DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
    DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
    DriveConstants.REAR_LEFT_CANCODER_ID,
    DriveConstants.REAR_LEFT_ZERO_ANGLE,
    DriveConstants.REAR_LEFT_CANCODER_REVERSED,
    DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
    "RL"
  );
  private final SwerveModule rearRightSwerveModule = new SwerveModule(
    DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
    DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
    DriveConstants.REAR_RIGHT_CANCODER_ID,
    DriveConstants.REAR_RIGHT_ZERO_ANGLE,
    DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
    DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
    "RR"
  );

  private final SwerveModule[] swerveModules = {
    frontLeftSwerveModule,
    frontRightSwerveModule,
    rearLeftSwerveModule,
    rearRightSwerveModule
  };

  private final SwerveDrivePoseEstimator odometry;

  private int gyroOffset = 0;
  private float rollOffset = 0;
  private float pitchOffset = 0;
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystemImpl() {
    odometry = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match
      stateStandardDeviations,
      visionMeasurementStandardDeviations
    );
  }

  @SuppressWarnings("ParameterName")
  @Override
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SmartDashboard.putBoolean("isFieldRelative", fieldRelative);
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getFieldRelativeRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  @Override
  public double getHeading() {
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  @Override
  public double getRoll() {
    return gyro.getRoll() - rollOffset;
  }

  @Override
  public double getPitch() {
    return gyro.getPitch() - pitchOffset;
  }

  @Override
  public double getBalanceError() {
    double roll = getRoll();
    double pitch = getPitch();
    double yaw = getHeading();
    return (pitch * Math.cos(yaw)) + (roll * Math.sin(yaw));
  }

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public Rotation2d getFieldRelativeRotation2d() {
    // Because the field isn't vertically symmetrical, we have the pose
    // coordinates always start from the bottom left
    return Rotation2d.fromDegrees((getHeading() + (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)) % 360);
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
  public void zeroPitchAndRoll() {
    pitchOffset = gyro.getPitch();
    rollOffset = gyro.getRoll();
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
  public void resetOdometry(Pose2d pose) {
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
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND); // TODO: Check if this has to be different
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(desiredStates[i]);
    }
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };

    return swerveModulePositions;
  }
  
  @Override
  public void periodic() {
    // Uses the swerve's sensors to update the pose estimator
    odometry.update(
      getRotation2d(),
      getModulePositions()
    );

    frontLeftSwerveModule.periodicFunction();
    rearLeftSwerveModule.periodicFunction();
    frontRightSwerveModule.periodicFunction();
    rearRightSwerveModule.periodicFunction();
  }

}