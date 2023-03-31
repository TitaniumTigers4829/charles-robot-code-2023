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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.commands.DriveCommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowPathPlannerTrajectory extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final LEDSubsystem leds;
  private final boolean resetOdometryToTrajectoryStart;
  
  private PPSwerveControllerCommand followTrajectoryCommand;
  private Pose2d trajectoryInitialPose;
  
  /* EDIT CODE BELOW HERE */
  // You should have constants for everything in here

  private final SwerveDriveKinematics driveKinematics = DriveConstants.DRIVE_KINEMATICS;
  
  private final double autoMaxVelocity = TrajectoryConstants.MAX_SPEED;
  private final double autoMaxAcceleration = TrajectoryConstants.MAX_ACCELERATION - .5;

  // Your probably only want to edit the P values
  private final PIDController xController = new PIDController(TrajectoryConstants.DEPLOYED_X_CONTROLLER_P, 0, 0);
  private final PIDController yController = new PIDController(TrajectoryConstants.DEPLOYED_Y_CONTROLLER_P, 0, 0);
  private final PIDController thetaController = new PIDController(TrajectoryConstants.DEPLOYED_THETA_CONTROLLER_P, 0, 0);
  
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
  public FollowPathPlannerTrajectory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, LEDSubsystem leds, String trajectoryName, boolean resetOdometryToTrajectoryStart) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;    
    this.leds = leds;
    addRequirements(visionSubsystem, leds);
    this.resetOdometryToTrajectoryStart = resetOdometryToTrajectoryStart;

    // Makes a trajectory                                                     
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(trajectoryName, 3, 1.5);

    trajectoryInitialPose = trajectoryToFollow.getInitialHolonomicPose();

    // Makes it so wheels don't have to turn more than 90 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    followTrajectoryCommand = new PPSwerveControllerCommand(
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
  }

  /**
   * Follows the specified PathPlanner trajectory.
   * @param driveSubsystem The subsystem for the swerve drive.
   * @param visionSubsystem The subsystem for vision measurements
   * @param trajectoryName The name of the PathPlanner path file. It should not include the filepath or 
   * .path extension.
   */
  public FollowPathPlannerTrajectory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, LEDSubsystem leds, String trajectoryName) {
    this(driveSubsystem, visionSubsystem, leds, trajectoryName, false);
  }

  @Override
  public void initialize() {

    leds.setProcess(LEDProcess.LINE_UP);

    if (resetOdometryToTrajectoryStart) {
      driveSubsystem.resetOdometryAndRotation(trajectoryInitialPose, trajectoryInitialPose.getRotation().getDegrees());
    }

    followTrajectoryCommand.schedule();
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.DEFAULT);
  }

  @Override
  public boolean isFinished() {
    return followTrajectoryCommand.isFinished();
  }
}