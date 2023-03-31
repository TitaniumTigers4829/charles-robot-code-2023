// JackLib 2023

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.commands.DriveCommandBase;
import frc.robot.extras.SmartDashboardLogger;
import frc.robot.subsystems.arm.ArmSubsystem;
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

public class AutoPlace extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final LEDSubsystem leds;
  private final BooleanSupplier isFinished;
  private Pose2d endPose;
  private double armRotation;
  private double armExtension;

  /**
   * Makes the robot drive to the specified node and place its cargo.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param isFinished The boolean supplier that returns true if the trajectory should be finished.
   */
  public AutoPlace(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, LEDSubsystem leds, BooleanSupplier isFinished) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.armSubsystem = armSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.leds = leds;
    // Doesn't require the drive subsystem because RealTimePPSwerveControllerCommand does
    addRequirements(visionSubsystem, armSubsystem, clawSubsystem, leds);
    this.isFinished = isFinished;
  }

  @Override
  public void initialize() {
    // The start of the trajectory is the robot's current location
    List<PathPoint> pathPoints = new ArrayList<PathPoint>();
    Translation2d start = new Translation2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY());
    Rotation2d startRotation = driveSubsystem.getPose().getRotation();
    pathPoints.add(new PathPoint(start, Rotation2d.fromDegrees(0), startRotation));
    // .withControlLengths(0.001, 0.001));)
    double endX;
    double endY;
    Rotation2d endRotation;

    // The middle waypoints and end pos change depending on alliance
    if (DriverStation.getAlliance() == Alliance.Blue) {
      endX = TrajectoryConstants.BLUE_NODE_X_POSITION;
      endY = TrajectoryConstants.BLUE_NODE_Y_POSITIONS[(driveSubsystem.getSelectedNode() - 1) % 9]; // NodeID starts at  1
      endRotation = TrajectoryConstants.BLUE_END_ROTATION;
      // Makes waypoints in the trajectory so the robot doesn't hit the charging station
      if (driveSubsystem.getPose().getX() > TrajectoryConstants.BLUE_OUTER_WAYPOINT_X) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_OUTER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_OUTER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));        }
      } else if (driveSubsystem.getPose().getX() > TrajectoryConstants.BLUE_INNER_WAYPOINT_X && driveSubsystem.getPose().getY() > TrajectoryConstants.UPPER_WAYPOINT_Y && driveSubsystem.getPose().getY() < TrajectoryConstants.LOWER_WAYPOINT_Y) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), endRotation, endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.BLUE_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), endRotation, endRotation));
        }
      }
    } else {
      endX = TrajectoryConstants.RED_NODE_Y_POSITION;
      endY = TrajectoryConstants.RED_NODE_Y_POSITIONS[(driveSubsystem.getSelectedNode() - 1) % 9];
      endRotation = TrajectoryConstants.RED_END_ROTATION;
      if (driveSubsystem.getPose().getX() < TrajectoryConstants.RED_OUTER_WAYPOINT_X) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_OUTER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), Rotation2d.fromDegrees(0), endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), Rotation2d.fromDegrees(0), endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_OUTER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), Rotation2d.fromDegrees(0), endRotation));
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), Rotation2d.fromDegrees(0), endRotation));
        }
      } else if (driveSubsystem.getPose().getX() < TrajectoryConstants.RED_INNER_WAYPOINT_X && driveSubsystem.getPose().getY() > TrajectoryConstants.UPPER_WAYPOINT_Y && driveSubsystem.getPose().getY() < TrajectoryConstants.LOWER_WAYPOINT_Y) {
        if (driveSubsystem.getPose().getY() - TrajectoryConstants.LOWER_WAYPOINT_Y > (TrajectoryConstants.UPPER_WAYPOINT_Y - TrajectoryConstants.LOWER_WAYPOINT_Y) / 2) {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.UPPER_WAYPOINT_Y), Rotation2d.fromDegrees(0), endRotation));
        } else {
          pathPoints.add(new PathPoint(new Translation2d(TrajectoryConstants.RED_INNER_WAYPOINT_X, TrajectoryConstants.LOWER_WAYPOINT_Y), Rotation2d.fromDegrees(0), endRotation));
        }
      }
    }

    Translation2d end = new Translation2d(endX, endY);

    SmartDashboard.putString("End", end.toString());

    pathPoints.add(new PathPoint(end, Rotation2d.fromDegrees(0), endRotation));

    endPose = new Pose2d(endY, endY, endRotation);

    // Gets rotation and extension based off of if scoring high, mid, low
    if (driveSubsystem.getSelectedNode() >= 18) {
      armRotation = ArmConstants.PLACE_HIGH_ROTATION;
      armExtension = ArmConstants.PLACE_HIGH_EXTENSION;
    } else if (driveSubsystem.getSelectedNode() >= 18) {
      armRotation = ArmConstants.PLACE_MIDDLE_ROTATION;
      armExtension = ArmConstants.PLACE_MIDDLE_EXTENSION;
    } else {
      armRotation = ArmConstants.PLACE_LOW_ROTATION;
      armExtension = ArmConstants.PLACE_LOW_EXTENSION;
    }

    // You probably only want to edit the P values
    PIDController xController = new PIDController(TrajectoryConstants.DEPLOYED_X_CONTROLLER_P, 0, 0);
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

      SmartDashboard.putString("end state", trajectoryToFollow.getEndState().toString());

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

    if (clawSubsystem.isConeMode()) {
      leds.setProcess(LEDProcess.SCORING_CONE);
    } else {
      leds.setProcess(LEDProcess.SCORING_CUBE);
    }
  }

  @Override
  public void execute() {
    super.execute();
    // if ((DriverStation.getAlliance() == Alliance.Blue && driveSubsystem.getPose().getX() < TrajectoryConstants.BLUE_INNER_WAYPOINT_X)
    //   || (DriverStation.getAlliance() == Alliance.Red && driveSubsystem.getPose().getX() > TrajectoryConstants.RED_INNER_WAYPOINT_X)) {
    //     armSubsystem.setRotation(armRotation);
    //     armSubsystem.setExtension(armExtension);
    //     if (clawSubsystem.isConeMode()) {
    //       clawSubsystem.setWristPosition(180);
    //     }
    // }

    // if (Math.abs(driveSubsystem.getPose().getX() - endPose.getX()) < TrajectoryConstants.X_TOLERANCE + .01
    //   && Math.abs(driveSubsystem.getPose().getY() - endPose.getY()) < TrajectoryConstants.Y_TOLERANCE + .01
    //   && Math.abs(driveSubsystem.getPose().getRotation().getDegrees() - endPose.getRotation().getDegrees()) < TrajectoryConstants.THETA_TOLERANCE + 1) {
    //     clawSubsystem.open();
    //     if (clawSubsystem.isConeMode()) {
    //       clawSubsystem.setIntakeSpeed(ClawConstants.PLACE_CONE_INTAKE_SPEED);
    //     } else {
    //       clawSubsystem.setIntakeSpeed(ClawConstants.PLACE_CUBE_INTAKE_SPEED);
    //     }
    //   }
  }

  @Override
  public void end(boolean interrupted) {
    // armSubsystem.setRotation(ArmConstants.STOWED_ROTATION);
    // armSubsystem.setExtension(ArmConstants.STOWED_EXTENSION);
    leds.setProcess(LEDProcess.DEFAULT);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}