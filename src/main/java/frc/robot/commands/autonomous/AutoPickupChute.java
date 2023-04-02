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
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.commands.DriveCommandBase;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.extras.SmartDashboardLogger;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AutoPickupChute extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final LEDSubsystem leds;
  private final BooleanSupplier isFinished;
  
  private Pose2d endPose;

  /**
   * Makes the robot drive to the specified node and place its cargo.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param isFinished The boolean supplier that returns true if the trajectory should be finished.
   */
  public AutoPickupChute(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, LEDSubsystem leds, BooleanSupplier isFinished) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.leds = leds;
    // Doesn't require the drive subsystem because RealTimePPSwerveControllerCommand does
    addRequirements(visionSubsystem, leds);
    this.isFinished = isFinished;
  }

  @Override
  public void initialize() {

    leds.setProcess(LEDProcess.LINE_UP);

    // The start of the trajectory is the robot's current location
    List<PathPoint> pathPoints = new ArrayList<PathPoint>();
    Translation2d start = new Translation2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY());
    Rotation2d startRotation = driveSubsystem.getPose().getRotation();
    Rotation2d trajectoryHeading = DriverStation.getAlliance() == Alliance.Blue
      ? TrajectoryConstants.BLUE_HEADING : TrajectoryConstants.RED_HEADING;
    pathPoints.add(new PathPoint(start, startRotation));
    double endX;
    double endY;
    Rotation2d endRotation;

    // The middle waypoints and end pos change depending on alliance
    if (DriverStation.getAlliance() == Alliance.Blue) {
      endX = TrajectoryConstants.BLUE_NODE_X_POSITION;
      endY = TrajectoryConstants.BLUE_NODE_Y_POSITIONS[(NodeAndModeRegistry.getSelectedNode() - 1) % 9]; // NodeID starts at  1
      endRotation = TrajectoryConstants.BLUE_END_ROTATION;
     } else {
      endX = 2.14;
      endY = 7.43;
      endRotation = Rotation2d.fromDegrees(90);
    }

    Translation2d end = new Translation2d(endX, endY);
    endPose = new Pose2d(endX, endY, endRotation);

    pathPoints.add(new PathPoint(end, endRotation));

    // You probably only want to edit the P values
    PIDController xController = new PIDController(TrajectoryConstants.REAL_TIME_X_CONTROLLER_P, 0, 0);
    PIDController yController = new PIDController(TrajectoryConstants.REAL_TIME_Y_CONTROLLER_P, 0, 0);
    PIDController thetaController = new PIDController(TrajectoryConstants.REAL_TIME_THETA_CONTROLLER_P, 0, 0);

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
        endPose,
        driveSubsystem
      ).schedule();
    } catch(Exception e) {
      SmartDashboardLogger.errorString("Trajectory Error Message", e.getLocalizedMessage());
    }
    SmarterDashboardRegistry.updateIsFinished(false);
  }

  @Override
  public void execute() {
    super.execute();
    if (Math.abs(endPose.getX() - driveSubsystem.getPose().getX()) < TrajectoryConstants.X_TOLERANCE
      && Math.abs(endPose.getY() - driveSubsystem.getPose().getY()) < TrajectoryConstants.Y_TOLERANCE
      && Math.abs(endPose.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees()) < TrajectoryConstants.THETA_TOLERANCE) {
        leds.setProcess(LEDProcess.FINISH_LINE_UP);
      }
  }

  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.DEFAULT);
    SmarterDashboardRegistry.updateIsFinished(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}