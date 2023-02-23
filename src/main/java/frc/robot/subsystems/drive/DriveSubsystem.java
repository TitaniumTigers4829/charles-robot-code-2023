package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveSubsystem extends Subsystem {
  
  /**
   * Drives the robot using the joysticks.
   * @param xSpeed Speed of the robot in the x direction, positive being 
   * forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being
   * left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative
   * to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, 
    boolean fieldRelative);

  /**
   * Returns the heading of the robot in degrees from 0 to 360. 
   * Counter-clockwise is positive. This factors in gyro offset.
   */
  public double getHeading();

  /**
   * Returns a Rotation2d for the heading of the robot.
   */
  public Rotation2d getRotation2d();

  /**
   * Sets the offset of the gyro.
   * @param gyroOffset The number of degrees that will be added to the
   * gyro's angle in getHeading.
   */
  public void setGyroOffset(int gyroOffset);

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading();

  /**
   * Returns the estimated field-relative pose of the robot. Positive x 
   * being forward, positive y being left.
   */
  public Pose2d getPose();

  /**
   * Resets the odometry to the specified pose, but keeps the current 
   * rotation.
   */
  public void resetOdometry(Pose2d pose);

  /**
   * Returns the current drivetrain position, as reported by the modules 
   * themselves. The order is: frontLeft, frontRight, backLeft, backRight
   * (should be the same as the kinematics).
   */
  public SwerveModulePosition[] getModulePositions();
  
  /**
   * Sets the modules to the specified states.
   * @param desiredStates The desired states for the swerve modules. The
   * order is: frontLeft, frontRight, backLeft, backRight (should be the 
   * same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates);
}