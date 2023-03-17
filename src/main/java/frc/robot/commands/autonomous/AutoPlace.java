// JackLib 2023

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.DriveCommandBase;
import frc.robot.dashboard.SmartDashboardLogger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AutoPlace extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final BooleanSupplier isFinished;
  private final int nodeID;

  /**
   * Makes the robot drive to the specified node and place its cargo.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param isFinished The boolean supplier that returns true if the trajectory should be finished.
   * @param nodeID The ID starting at index 1 for the node to place something at.
   */
  public AutoPlace(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished, int nodeID) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
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
      endX = TrajectoryConstants.BLUE_NODE_X_POSITION;
      endY = TrajectoryConstants.BLUE_NODE_Y_POSITIONS[(nodeID % 9) - 1]; // NodeID starts at  1
      endRotation = TrajectoryConstants.BLUE_END_ROTATION;
      // Makes waypoints in the trajectory so the robot doesn't hit the charging station
      if (driveSubsystem.getPose().getX() > TrajectoryConstants.BLUE_OUTER_WAYPOINT_X) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_OUTER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_OUTER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));        }
      } else if (driveSubsystem.getPose().getX() > TrajectoryConstants.BLUE_INNER_WAYPOINT_X) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));
        }
      }
    } else {
      endX = TrajectoryConstants.RED_NODE_Y_POSITION;
      endY = TrajectoryConstants.RED_NODE_Y_POSITIONS[(nodeID % 9) - 1];
      endRotation = TrajectoryConstants.RED_END_ROTATION;
      if (driveSubsystem.getPose().getX() < TrajectoryConstants.RED_OUTER_WAYPOINT_X) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_OUTER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_OUTER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));        }
      } else if (driveSubsystem.getPose().getX() > TrajectoryConstants.RED_INNER_WAYPOINT_X) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y < (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));
        }
      }
    }

    Translation2d end = new Translation2d(endX, endY);

    pathPoints.add(new PathPoint(end, endRotation, endRotation));

    // You probably only want to edit the P values
    PIDController xController = new PIDController(TrajectoryConstants.X_CONTROLLER_P, 0, 0);
    PIDController yController = new PIDController(TrajectoryConstants.Y_CONTROLLER_P, 0, 0);
    PIDController thetaController = new PIDController(TrajectoryConstants.THETA_CONTROLLER_P, 0, 0);

    // This should be fine, but is here just in case so the robot doesn't crash during a match
    try {
      // Makes a trajectory that factors in holonomic rotation
      PathPlannerTrajectory trajectoryToFollow = PathPlanner.generatePath(
        new PathConstraints(TrajectoryConstants.MAX_SPEED, TrajectoryConstants.MAX_ACCELERATION),
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
        DriveConstants.DRIVE_KINEMATICS,
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
      SmartDashboardLogger.errorString("Trajectory Error Message", e.getLocalizedMessage());
    }
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

}