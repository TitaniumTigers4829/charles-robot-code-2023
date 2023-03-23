// JackLib 2023

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.DriveCommandBase;
import frc.robot.extras.SmartDashboardLogger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AutoPickup extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final BooleanSupplier isFinished;

  /**
   * Makes the robot drive to the specified node and place its cargo.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param isFinished The boolean supplier that returns true if the trajectory should be finished.
   */
  public AutoPickup(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    // Doesn't require the drive subsystem because RealTimePPSwerveControllerCommand does
    addRequirements(visionSubsystem);
    this.isFinished = isFinished;
  }

  @Override
  public void initialize() {
    // Try making it do a path with hard coded setpoints
    // Make sure when the gyro is zeroes it's right

    // The start of the trajectory is the robot's current location
    List<PathPoint> pathPoints = new ArrayList<PathPoint>();
    Translation2d start = new Translation2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY());
    Rotation2d startRotation = driveSubsystem.getPose().getRotation();
    pathPoints.add(new PathPoint(start, startRotation, startRotation));
    double endX;
    double endY;
    Rotation2d endRotation;

    endX = start.getX();
    endY = start.getY();
    endRotation = startRotation;

    Translation2d end = new Translation2d(endX, endY);

    pathPoints.add(new PathPoint(end, endRotation, endRotation));

    SmartDashboard.putString("start", start.toString());
    SmartDashboard.putString("end", end.toString());

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

      System.out.println(trajectoryToFollow.getStates());

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