package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.LimelightResults;

public class VisionSubsystemImpl extends SubsystemBase implements VisionSubsystem {
  
  // TODO: This only works for one LL, will have to change methods for both
  public VisionSubsystemImpl() {}

  @Override
  public void periodic() {}

  @Override
  public boolean canSeeAprilTags() {
    LimelightResults limelightResults = LimelightHelpers.getLatestResults(LimelightConstants.frontLimelightName);
    return limelightResults.targetingResults.targets_Fiducials.length > 0;
  }

  @Override
  public Pose2d getPoseFromAprilTags() {
    Pose2d botPose = LimelightHelpers.getBotPose2d(LimelightConstants.frontLimelightName);
    double robotX = botPose.getX() + TrajectoryConstants.fieldLengthMeters / 2;
    double robotY = botPose.getY() + TrajectoryConstants.fieldWidthMeters / 2;
    Rotation2d robotRotation = botPose.getRotation();
    return new Pose2d(robotX, robotY, robotRotation);
  }

  @Override
  public double getDistanceFromClosestAprilTag() {
    if (canSeeAprilTags()) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(LimelightConstants.frontLimelightName);
      double aprilTagX = LimelightConstants.aprilTagPositions[closestAprilTagID - 1][0]; // April tag id starts at 1
      double aprilTagY = LimelightConstants.aprilTagPositions[closestAprilTagID - 1][1];
      double robotX = getPoseFromAprilTags().getX();
      double robotY = getPoseFromAprilTags().getY();
      return Math.sqrt(Math.pow(aprilTagX - robotX, 2) + Math.pow(aprilTagY - robotY, 2));
    }
    
    // To be safe returns a big distance from the april tags
    return 10;
  }

  @Override
  public int getNumberOfAprilTags() {
    LimelightResults limelightResults = LimelightHelpers.getLatestResults(LimelightConstants.frontLimelightName);
    return limelightResults.targetingResults.targets_Fiducials.length;
  }

  @Override
  public long getTimeStampSeconds() {
    LimelightResults limelightResults = LimelightHelpers.getLatestResults(LimelightConstants.frontLimelightName);
    return (long) (limelightResults.targetingResults.timestamp_LIMELIGHT_publish / 1000);
  }

  @Override
  public boolean canSeeCube() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Translation2d getClosestCubePosition() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public boolean canSeeCone() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Translation2d getClosestConePosition() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void cropLimelights(double[][] cropValues) {
    LimelightHelpers.setCropWindow(
      LimelightConstants.frontLimelightName, 
      cropValues[0][0],
      cropValues[0][1],
      cropValues[0][2],
      cropValues[0][3]
    );
  }

  @Override
  public void setLimelightsPipeline(LimelightPipelines limelightPipeline) {
    LimelightHelpers.setPipelineIndex(LimelightConstants.frontLimelightName, limelightPipeline.getID());
  }

}