package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.LimelightResults;
import frc.robot.extras.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystemImpl extends SubsystemBase implements VisionSubsystem {

  private LimelightResults currentlyUsedLimelightResults;
  private String currentlyUsedLimelight = LimelightConstants.FRONT_LIMELIGHT_NAME;
  
  public VisionSubsystemImpl() {
    try {
      currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(LimelightConstants.FRONT_LIMELIGHT_NAME);
    } catch (Exception e) {
      currentlyUsedLimelightResults = null;
    }
  }

  @Override
  public void periodic() {
    try {
      // Every periodic chooses the limelight to use based off of their distance from april tags
      LimelightTarget_Fiducial[] frontLimelightAprilTags = LimelightHelpers.getLatestResults(LimelightConstants.FRONT_LIMELIGHT_NAME).targetingResults.targets_Fiducials;
      LimelightTarget_Fiducial[] backLimelightAprilTags = LimelightHelpers.getLatestResults(LimelightConstants.BACK_LIMELIGHT_NAME).targetingResults.targets_Fiducials;

      // Gets the distance from the closest april tag. If it can't see one, returns a really big number.
      double frontLimelightDistance = frontLimelightAprilTags.length > 0
        ? getLimelightAprilTagDistance((int) frontLimelightAprilTags[0].fiducialID) : Double.MAX_VALUE;
      double backLimelightDistance = backLimelightAprilTags.length > 0
        ? getLimelightAprilTagDistance((int) backLimelightAprilTags[0].fiducialID) : Double.MAX_VALUE;

      currentlyUsedLimelight = frontLimelightDistance <= backLimelightDistance 
        ? LimelightConstants.FRONT_LIMELIGHT_NAME : LimelightConstants.BACK_LIMELIGHT_NAME;
      currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(currentlyUsedLimelight);
      SmartDashboard.putString("currentlyUsedLimelight", currentlyUsedLimelight);
      SmartDashboard.putString("limelight pose", getPoseFromAprilTags().toString());
    } catch (Exception e) {
      SmartDashboard.putString("currentlyUsedLimelight", "Limelight err: " + e.toString());
    }
  }

  @Override
  public boolean canSeeAprilTags() {
    try {
      return LimelightHelpers.getFiducialID(currentlyUsedLimelight) != -1;
    } catch (Exception e) {
      return false;
    }
  }

  @Override
  public Pose2d getPoseFromAprilTags() {
    try {
      Pose2d botPose = LimelightHelpers.getBotPose2d(currentlyUsedLimelight);
      // The origin of botpose is at the center of the field
      double robotX = botPose.getX() + TrajectoryConstants.FIELD_LENGTH_METERS / 2;
      double robotY = botPose.getY() + TrajectoryConstants.FIELD_WIDTH_METERS / 2;
      Rotation2d robotRotation = botPose.getRotation();
      return new Pose2d(robotX, robotY, robotRotation);
    } catch (Exception e) {
      // TODO: add logic here
      return null;
    }
  }

  @Override
  public double getDistanceFromClosestAprilTag() {
    if (canSeeAprilTags()) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(currentlyUsedLimelight);
      return getLimelightAprilTagDistance(closestAprilTagID);
    }
    
    // To be safe returns a big distance from the april tags
    return Double.MAX_VALUE;
  }

  @Override
  public int getNumberOfAprilTags() {
    try {
      return currentlyUsedLimelightResults.targetingResults.targets_Fiducials.length;
    } catch (Exception e) {
      return 0;
    }
  }

  @Override
  public long getTimeStampSeconds() {
    try {
      return (long) (currentlyUsedLimelightResults.targetingResults.timestamp_LIMELIGHT_publish / 1000);
    } catch (Exception e) {
      // TODO: add more logic
      return -1;
    }
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
      LimelightConstants.FRONT_LIMELIGHT_NAME, 
      cropValues[0][0],
      cropValues[0][1],
      cropValues[0][2],
      cropValues[0][3]
    );
  }

  @Override
  public void setLimelightsPipeline(LimelightPipelines limelightPipeline) {
    LimelightHelpers.setPipelineIndex(LimelightConstants.FRONT_LIMELIGHT_NAME, limelightPipeline.getID());
  }

  /**
   * Calculates the distance between the specified robot and april tag.
   * This method should only be called once there has been a check for if
   * the limelights can see april tags.
   */
  private double getLimelightAprilTagDistance(int aprilTagID) {
    if (aprilTagID >= 1) {
      double aprilTagX = LimelightConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][0]; // April tag id starts at 1
      double aprilTagY = LimelightConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][1];
      // Added a little optimization
      Pose2d pose = getPoseFromAprilTags();
      double robotX = pose.getX();
      double robotY = pose.getY();
      return Math.sqrt(Math.pow(aprilTagX - robotX, 2) + Math.pow(aprilTagY - robotY, 2));
    }

    // To be safe returns a big distance from the april tags
    return Double.MAX_VALUE;
  }

}
