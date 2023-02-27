// JackLib 2023
// For this code to work, PathPlannerLib needs to be installed through Gradle
// https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowPathPlannerTrajectory extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final String trajectoryName;
  private final boolean resetOdometryToTrajectoryStart;
  
  private PPSwerveControllerCommand followPathPlannerTrajectoryCommand;
  private boolean done = false;
  private double consecutiveAprilTagFrames = 0;
  private double lastTimeStampSeconds = 0;
  
  /* EDIT CODE BELOW HERE */
  // You should have constants for everything in here

  private final SwerveDriveKinematics driveKinematics = DriveConstants.driveKinematics;
  
  private final double autoMaxVelocity = TrajectoryConstants.autoMaxVelocity;
  private final double autoMaxAcceleration = TrajectoryConstants.autoMaxAcceleration;

  // Your probably only want to edit the P values
  private final PIDController xController = new PIDController(TrajectoryConstants.xControllerP, 0, 0);
  private final PIDController yController = new PIDController(TrajectoryConstants.yControllerP, 0, 0);
  private final PIDController thetaController = new PIDController(TrajectoryConstants.thetaControllerP, 0, 0);
  
  // IMPORTANT: Make sure your driveSubsystem has the methods resetOdometry, getPose, and setModuleStates
  
  /* EDIT CODE ABOVE HERE (ONLY TOUCH THE REST OF THE CODE IF YOU KNOW WHAT YOU'RE DOING) */

  /**
   * Follows the specified PathPlanner trajectory.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param trajectoryName The name of the PathPlanner path file. It should not include the filepath or 
   * .path extension.
   * @param resetOdometryToTrajectoryStart Set as true if you want the odometry of the robot to be set to the
   * start of the trajectory.
   */
  public FollowPathPlannerTrajectory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, String trajectoryName, boolean resetOdometryToTrajectoryStart) {
    this.driveSubsystem = driveSubsystem;    
    this.visionSubsystem = visionSubsystem;
    addRequirements(visionSubsystem);
    this.trajectoryName = trajectoryName;
    this.resetOdometryToTrajectoryStart = resetOdometryToTrajectoryStart;
  }

  @Override
  public void initialize() {
    // Makes a trajectory                                                     
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(trajectoryName, autoMaxVelocity, autoMaxAcceleration);

    // Makes it so wheels don't have to turn more than 90 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    if (resetOdometryToTrajectoryStart) {
      driveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose());
    }

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    followPathPlannerTrajectoryCommand = new PPSwerveControllerCommand(
      trajectoryToFollow,
      driveSubsystem::getPose, // Functional interface to feed supplier
      driveKinematics,
      xController,
      yController,
      thetaController,
      driveSubsystem::setModuleStates,
      false,
      driveSubsystem
    );
    
    followPathPlannerTrajectoryCommand.schedule();
  }

  @Override
  public void execute() {
    done = followPathPlannerTrajectoryCommand.isFinished();

    // Updates the robot's odometry with april tags
    double currentTimeStampSeconds = lastTimeStampSeconds;

    if (visionSubsystem.canSeeAprilTags()) {
      currentTimeStampSeconds = visionSubsystem.getTimeStampSeconds();
      consecutiveAprilTagFrames++;
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

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return done;
  }
}