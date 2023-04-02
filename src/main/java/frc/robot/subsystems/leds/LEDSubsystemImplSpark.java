// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.SparkConstants;
import frc.robot.extras.NodeAndModeRegistry;
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

  @Override
  public void periodic() {
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
