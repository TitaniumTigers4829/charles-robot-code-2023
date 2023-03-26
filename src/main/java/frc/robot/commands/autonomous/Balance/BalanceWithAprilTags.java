// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Balance;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.DriveConstants.BalanceConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemImpl;

public class BalanceWithAprilTags extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystemImpl visionSubsystem;
  private final boolean fromLeft;

  //Most likely won't be using this command :)
  Pose2d pose;
  double robotX = pose.getX();
  double robotY = pose.getY();

  private final PIDController balancePidController = new PIDController(
    BalanceConstants.BALANCE_P,
    BalanceConstants.BALANCE_I,
    BalanceConstants.BALANCE_D
  );

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    Constants.DriveConstants.FACEFORWARD_P, 
    0, 
    0, 
    TrajectoryConstants.THETA_CONTROLLER_CONSTRAINTS
  );


  
  /** 
   * Creates a new BalanceWithAprilTags.
   */
  public BalanceWithAprilTags(DriveSubsystem driveSubsystem, VisionSubsystemImpl visionSubsystem, boolean fromLeft) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.fromLeft = fromLeft;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (visionSubsystem.getDistanceFromClosestAprilTag() > 5
     //&& robotY != Constants.LimelightConstants.APRIL_TAG_POSITIONS[6]
     ) {
        driveForward(1.1, true);
    } else if (visionSubsystem.getDistanceFromClosestAprilTag() < 4) {
        driveForward(-1, true);
    } else {
        driveForward(0, false);
    }
        
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Drives the robot forward at a given rate.
   * @param faceForward whether or not to rotate the robot to orient itself forward.
   */
  private void driveForward(double speed, boolean faceForward) {
    double driveSpeed = speed;
    double rot = 0;

    if (fromLeft) {
      driveSpeed *= -1;
    }

    if (faceForward && !(Math.abs(driveSubsystem.getHeading()) < BalanceConstants.ORIENTATION_ERROR_CONSIDERED_ORIENTED)) {
      rot = thetaController.calculate(driveSubsystem.getHeading(), 0);
    }

    driveSubsystem.drive(
      driveSpeed,
      0,
      rot,
      true
    );
  }

}
