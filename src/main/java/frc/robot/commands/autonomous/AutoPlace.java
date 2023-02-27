// JackLib 2023

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AutoPlace extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final BooleanSupplier isFinished;
  private final int nodeID;

  private double consecutiveAprilTagFrames = 0;
  private double lastTimeStampSeconds = 0;

  /**
   * Makes the robot drive to the specified node and place its cargo.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param isFinished The boolean supplier that returns true if the trajectory should be finished.
   * @param nodeID The ID starting at index 1 for the node to place something at.
   */
  public AutoPlace(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished, int nodeID) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    // Doesn't require the drive subsystem because RealTimePPSwerveControllerCommand does
    addRequirements(visionSubsystem);
    this.isFinished = isFinished;
    this.nodeID = nodeID;
  }

  @Override
  public void initialize() {
    // The start of the trajectory is the robot's current location
    List<PathPoint> pathPoints = new ArrayList<PathPoint>();
    Translation2d start = new Translation2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY());
    Rotation2d startRotation = driveSubsystem.getPose().getRotation();
    pathPoints.add(new PathPoint(start, startRotation, startRotation));
    double endX;
    double endY;
    Rotation2d endRotation;

    // The middle waypoints and end pos change depending on alliance
    if (DriverStation.getAlliance() == Alliance.Blue) {
      endX = TrajectoryConstants.blueNodeXPosition;
      endY = TrajectoryConstants.blueNodeYPositions[(nodeID % 9) - 1]; // NodeID starts at  1
      endRotation = TrajectoryConstants.blueEndRotation;
      // Makes waypoints in the trajectory so the robot doesn't hit the charging station
      if (driveSubsystem.getPose().getX() > TrajectoryConstants.blueOuterWaypointX) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.lowerWaypointY > (TrajectoryConstants.upperWaypointY - TrajectoryConstants.lowerWaypointY) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.blueOuterWaypointX, TrajectoryConstants.upperWaypointY), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.blueInnerWaypointX, TrajectoryConstants.upperWaypointY), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.blueOuterWaypointX, TrajectoryConstants.lowerWaypointY), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.blueInnerWaypointX, TrajectoryConstants.lowerWaypointY), endRotation, endRotation));        }
      } else if (driveSubsystem.getPose().getX() > TrajectoryConstants.blueInnerWaypointX) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.lowerWaypointY > (TrajectoryConstants.upperWaypointY - TrajectoryConstants.lowerWaypointY) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.blueInnerWaypointX, TrajectoryConstants.upperWaypointY), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.blueInnerWaypointX, TrajectoryConstants.lowerWaypointY), endRotation, endRotation));
        }
      }
    } else {
      endX = TrajectoryConstants.redNodeXPosition;
      endY = TrajectoryConstants.redNodeYPositions[(nodeID % 9) - 1];
      endRotation = TrajectoryConstants.redEndRotation;
      if (driveSubsystem.getPose().getX() < TrajectoryConstants.redOuterWaypointX) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.lowerWaypointY > (TrajectoryConstants.upperWaypointY - TrajectoryConstants.lowerWaypointY) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.redOuterWaypointX, TrajectoryConstants.upperWaypointY), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.redInnerWaypointX, TrajectoryConstants.upperWaypointY), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.redOuterWaypointX, TrajectoryConstants.lowerWaypointY), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.redInnerWaypointX, TrajectoryConstants.lowerWaypointY), endRotation, endRotation));        }
      } else if (driveSubsystem.getPose().getX() > TrajectoryConstants.redInnerWaypointX) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.lowerWaypointY < (TrajectoryConstants.upperWaypointY - TrajectoryConstants.lowerWaypointY) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.redInnerWaypointX, TrajectoryConstants.upperWaypointY), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.redInnerWaypointX, TrajectoryConstants.lowerWaypointY), endRotation, endRotation));
        }
      }
    }

    Translation2d end = new Translation2d(endX, endY);

    pathPoints.add(new PathPoint(end, endRotation, endRotation));

    // You probably only want to edit the P values
    PIDController xController = new PIDController(TrajectoryConstants.xControllerP, 0, 0);
    PIDController yController = new PIDController(TrajectoryConstants.yControllerP, 0, 0);
    PIDController thetaController = new PIDController(TrajectoryConstants.thetaControllerP, 0, 0);

    // This should be fine, but is here just in case so the robot doesn't crash during a match
    try {
      // Makes a trajectory that factors in holonomic rotation
      PathPlannerTrajectory trajectoryToFollow = PathPlanner.generatePath(
        new PathConstraints(TrajectoryConstants.autoMaxVelocity, TrajectoryConstants.autoMaxAcceleration),
        // Pathpoints go in: position, heading (direction of travel)
        pathPoints
      );                                               

    // IMPORTANT: Make sure your driveSubsystem has the methods getPose and setModuleStates

    /* EDIT CODE ABOVE HERE (ONLY TOUCH THE REST OF THE CODE IF YOU KNOW WHAT YOU'RE DOING) */

      // Makes it so wheels don't have to turn more than 90 degrees
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      new RealTimePPSwerveControllerCommand(
        trajectoryToFollow,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.driveKinematics,
        xController,
        yController,
        thetaController,
        driveSubsystem::setModuleStates,
        false,
        isFinished,
        new Pose2d(endX, endY, endRotation),
        driveSubsystem
      ).schedule();
    } catch(Exception e) {
      SmartDashboard.putString("Trajectory Error Message", e.getLocalizedMessage());
    }
  }

  @Override
  public void execute() {
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
    return false;
  }

}