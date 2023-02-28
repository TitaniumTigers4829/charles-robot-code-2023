// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.extras.MultiLinearInterpolator;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveCommand extends CommandBase {

  private final MultiLinearInterpolator oneAprilTagLookupTable = 
    new MultiLinearInterpolator(LimelightConstants.oneAprilTagLookupTable);
  private final MultiLinearInterpolator twoAprilTagLookupTable = 
    new MultiLinearInterpolator(LimelightConstants.twoAprilTagLookupTable);

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final DoubleSupplier leftY, leftX, rightX;
  private final BooleanSupplier isFieldRelative;

  private double consecutiveAprilTagFrames = 0;
  private double lastTimeStampSeconds = 0;

  /**
   * The command for driving the robot using joystick inputs.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftY The joystick input for driving forward and backwards
   * @param leftX The joystick input for driving left and right
   * @param rightX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive
   * field relative
   */
  public DriveCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier isFieldRelative) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
    this.leftY = leftY;
    this.leftX = leftX;
    this.rightX = rightX;
    this.isFieldRelative = isFieldRelative;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Drives the robot
    driveSubsystem.drive(
      leftY.getAsDouble() * DriveConstants.joystickMaxSpeedMetersPerSecondLimit,
      leftX.getAsDouble() * DriveConstants.joystickMaxSpeedMetersPerSecondLimit,
      rightX.getAsDouble() * DriveConstants.maxAngularSpeedRadiansPerSecond,
      isFieldRelative.getAsBoolean()
    );

    // Updates the robot's odometry with april tags
    double currentTimeStampSeconds = lastTimeStampSeconds;

    if (visionSubsystem.canSeeAprilTags()) {
      currentTimeStampSeconds = visionSubsystem.getTimeStampSeconds();
      consecutiveAprilTagFrames++;

      double distanceFromClosestAprilTag = visionSubsystem.getDistanceFromClosestAprilTag();
      // Sets the pose estimator confidence in vision based off of number of april tags and distance
      if (visionSubsystem.getNumberOfAprilTags() == 1) {
        double xStandardDeviation = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag)[0];
        double yStandardDeviation = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag)[1];
        double thetaStandardDeviation = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag)[2];
        driveSubsystem.setPoseEstimatorVisionConfidence(xStandardDeviation, yStandardDeviation, thetaStandardDeviation);
      } else if (visionSubsystem.getNumberOfAprilTags() > 1) {
        double xStandardDeviation = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag)[0];
        double yStandardDeviation = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag)[1];
        double thetaStandardDeviation = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag)[2];
        driveSubsystem.setPoseEstimatorVisionConfidence(xStandardDeviation, yStandardDeviation, thetaStandardDeviation);
      }

      // Only updates the pose estimator if the limelight pose is new and reliable
      if (currentTimeStampSeconds > lastTimeStampSeconds && consecutiveAprilTagFrames > 2) {
        Pose2d limelightVisionMeasurement = visionSubsystem.getPoseFromAprilTags();
        driveSubsystem.addPoseEstimatorVisionMeasurement(limelightVisionMeasurement, currentTimeStampSeconds);
      }
    } else {
      consecutiveAprilTagFrames = 0;
    }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}