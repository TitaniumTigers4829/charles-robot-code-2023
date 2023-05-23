package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmarterDashboardRegistry {
  // alliance, node, isFinished
  // int (1 if red), int (1-27), int (1 if finished with path)
  private static double[] pathValues = new double[]{DriverStation.getAlliance() == Alliance.Blue ? 0 : 1, NodeAndModeRegistry.getSelectedNode(), 0};
  // x, y, pose
  private static double[] pose = new double[]{0, 0, 0};
  // x, y, pose
  private static double[] limelightPose = new double[]{0, 0, 0};
  // orientation
  // pitch, roll, yaw
  private static double[] orientation = new double[]{0, 0, 0};
  // is cone mode
  // true/false
  private static boolean isConeMode = NodeAndModeRegistry.isConeMode();

  private SmarterDashboardRegistry() {}

  /**
   * Sets the selected node in SmarterDashboard
   * @param nodeID the ID of the node
   */
  public static void setNode(double nodeID) {
    pathValues[1] = nodeID;
    SmartDashboard.putNumberArray("autoPlacePath", pathValues);
  }

  /**
   * Updates the alliance
   */
  public static void updateAlliance() {
    pathValues[0] = DriverStation.getAlliance() == Alliance.Blue ? 0 : 1;
    SmartDashboard.putNumberArray("autoPlacePath", pathValues);
  }

  /**
   * Updates if the path is finished
   * @param isFinished if the path is finished
   */
  public static void updateIsFinished(boolean isFinished) {
    pathValues[2] = isFinished ? 1 : 0;
    SmartDashboard.putNumberArray("autoPlacePath", pathValues);
  }

  /**
   * Sets the cone mode
   * @param isConeMode_ if it is cone mode
   */
  public static void setIsConeMode(boolean isConeMode_) {
    isConeMode = isConeMode_;
    SmartDashboard.putString("cargoMode", isConeMode ? "Cone" : "Cube");
  }

  // private static void pushValues() {
  //   SmartDashboard.putNumberArray("autoPlacePath", path_values);
  //   SmartDashboard.putString("cargoMode", isConeMode ? "Cone" : "Cube");
  //   SmartDashboard.putNumberArray("botAngle", orientation);
  //   SmartDashboard.putNumberArray("botPose", pose);
  //   SmartDashboard.putNumberArray("limelightPose", limelightPose);
  // }

  /**
   * Updates the smarterdashboard robot pose
   * @param robotPose the position of the robot
   */
  public static void setPose(Pose2d robotPose) {
    pose[0] = robotPose.getX();
    pose[1] = robotPose.getY();
    pose[2] = robotPose.getRotation().getDegrees();
    SmartDashboard.putNumberArray("botPose", pose);
  }

  /**
   * Gets the most recent estimated pose as updated by setPose(). A little scuffed but it works
   * @return the most recent pose
   */
  public static Pose2d getPose() {
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[2]));
  }

  /**
   * Sets the orientation for display in smarterdashboard
   * @param pitch the yaw (pitch and yaw are swapped)
   * @param roll the roll
   * @param yaw the pitch (pitch and yaw are swapped)
   */
  public static void setOrientation(double pitch, double roll, double yaw) {
    orientation[0] = pitch;
    orientation[1] = roll;
    orientation[2] = yaw;
    SmartDashboard.putNumberArray("botAngle", orientation);
  }

  /**
   * Updates the smarterdashboard limelight pose
   * @param limelightPose_ the estimated pose from the limelight
   */
  public static void setLimelightPose(Pose2d limelightPose_) {
    limelightPose[0] = limelightPose_.getX();
    limelightPose[1] = limelightPose_.getY();
    limelightPose[2] = limelightPose_.getRotation().getDegrees();
    SmartDashboard.putNumberArray("limelightPose", limelightPose);
  }

  /**
   * Not used (yet). Returns the most recent limelight pose as updated by setLimelightPose()
   * @return the most recent limelight pose
   */
  public static Pose2d getLimelightPose() {
    return new Pose2d(limelightPose[0], limelightPose[1], Rotation2d.fromDegrees(limelightPose[2]));
  }
}
