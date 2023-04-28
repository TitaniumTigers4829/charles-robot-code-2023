// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.SparkConstants;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.Constants.LEDConstants.LEDProcess;


public class LEDSubsystemImplSpark extends SubsystemBase implements LEDSubsystem {

  private Spark ledSpark;

  private LEDProcess process;

  /** Creates a new LEDSubsystemImpl for use with LED strips made by Spark lighting (Loopy's LEDs).
   * @param port The Spark port for this LEDSubsystem.
  */
  public LEDSubsystemImplSpark(int port) {
    ledSpark = new Spark(port);
    setProcess(LEDProcess.OFF);
  }

  /** Creates a new LEDSubsystemImpl with the port in LEDConstants. */
  public LEDSubsystemImplSpark() {
    ledSpark = new Spark(LEDConstants.LEDPort);
    setProcess(LEDProcess.OFF);
  }

  private static boolean inChuteZone(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return x > DriveConstants.BLUE_CHUTE_THRESHOLD_X && y > DriveConstants.BLUE_CHUTE_THRESHOLD_Y;
    } else {
      return x < DriveConstants.RED_CHUTE_THRESHOLD_X && y > DriveConstants.RED_CHUTE_THRESHOLD_Y;
    }
  }

  @Override
  public void periodic() {
    Pose2d currentPose = SmarterDashboardRegistry.getPose();
    if (inChuteZone(currentPose)) {
      double currentX = currentPose.getX();
      double chuteX = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? DriveConstants.BLUE_CHUTE_X : DriveConstants.RED_CHUTE_X;
      double error = Math.abs(currentX - chuteX);
      if (error <= DriveConstants.LEDS_ACCEPTABLE_ERROR) {
        this.setProcess(LEDProcess.GREEN);
      } else {
        this.setProcess(LEDProcess.RED);
      }
    } else {
      if (process == LEDProcess.GREEN || process == LEDProcess.RED) {
        this.setProcess(LEDProcess.DEFAULT);
      }
    }
    ledSpark.set(getSparkFromProcess(process));
  }

  @Override
  public void setProcess(LEDProcess process) {
    this.process = process;
  }

  private double getSparkFromProcess(LEDProcess pr) {
    switch (pr) {
      case DEFAULT:
        return defaultColor();
      case ALLIANCE_COLOR:
        return allianceColor();
      default:
        return pr.getSparkValue();
    }
  }

  private double cargoMode() {
    if (NodeAndModeRegistry.isConeMode()) {
      return LEDProcess.CONE.getSparkValue();
    } else {
      return LEDProcess.CUBE.getSparkValue();
    }
  }

  private double allianceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        return LEDProcess.RED_ALLIANCE.getSparkValue();
    } else {
        return LEDProcess.BLUE_ALLIANCE.getSparkValue();
    }
  }

  private double defaultColor() {
    if (DriverStation.isAutonomous()) {
      return LEDProcess.AUTONOMOUS.getSparkValue();
    } else {
      return cargoMode();
    }
  }

  @Override
  public void off() {
    process = LEDProcess.OFF;
  }

}
