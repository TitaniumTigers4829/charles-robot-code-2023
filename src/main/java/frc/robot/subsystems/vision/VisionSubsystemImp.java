package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface VisionSubsystemImp {
  
  /**
   * Returns true if the limelight(s) can see one or more April Tag.
   */
  public boolean canSeeAprilTags();

  /**
   * Returns the pose of the robot calculated by the limelight. If there
   * are multiple limelights that can see april tags, it uses the limelight
   * that is closest to an april tag. 
   */
  public Pose2d getPoseFromAprilTags();

  /**
   * Returns the timestamp in seconds of when the limelight calculated 
   * the robot's pose. If there are multiple limelights that can see april 
   * tags, it uses the limelight that is closest to an april tag. 
   */
  public double getTimeStampInSeconds();

  /**
   * Returns true if the limelight(s) can see one or more cubes.
   */
  public boolean canSeeCube();

  /**
   * Returns the translation of the closest cube relative to the robot.
   * Positive x being forward, positive y being left.
   */
  public Translation2d getClosestCubePosition();

  /**
   * Returns true if the limelight(s) can see one or more cones.
   */
  public boolean canSeeCone();

  /**
   * Returns the translation of the closest cone relative to the robot.
   * Positive x being forward, positive y being left.
   */
  public Translation2d getClosestConePosition();

}
