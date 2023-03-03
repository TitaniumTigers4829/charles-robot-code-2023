// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.extras.MultiLinearInterpolator;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public abstract class DriveCommandBase extends CommandBase {

  private final MultiLinearInterpolator oneAprilTagLookupTable = 
    new MultiLinearInterpolator(LimelightConstants.oneAprilTagLookupTable);
  private final MultiLinearInterpolator twoAprilTagLookupTable = 
    new MultiLinearInterpolator(LimelightConstants.twoAprilTagLookupTable);

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double consecutiveAprilTagFrames = 0;
  private double lastTimeStampSeconds = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   */
  public DriveCommandBase(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void execute() {
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
      if (currentTimeStampSeconds > lastTimeStampSeconds && consecutiveAprilTagFrames > LimelightConstants.detectedFramesForReliability) {
        Pose2d limelightVisionMeasurement = visionSubsystem.getPoseFromAprilTags();
        driveSubsystem.addPoseEstimatorVisionMeasurement(limelightVisionMeasurement, currentTimeStampSeconds);
      }
    } else {
      consecutiveAprilTagFrames = 0;
    }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }

}