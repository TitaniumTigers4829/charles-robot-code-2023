// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.SparkConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;


public class LEDSubsystemImplSpark extends SubsystemBase implements LEDSubsystem {

  private Spark ledSpark;

  private double currentSparkValue;

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
    ledSpark.set(getCurrentPipelineIndex);
  }

  /** Sets the pattern to a double value.
   * @param value The pattern to set the LEDs to. 
   * See Constants.LEDConstants.SparkConstants for these pattern values.
   */
  public void setSpark(double value) {
    // If the value is a valid Spark Value.
    if (-1 < value && value < 1 && ((value*100)%2) == 1) {
      currentSparkValue = value;
    }
  }

  @Override
  public void setProcess(LEDProcess process) {
    switch (process) {
      case DEFAULT:
        currentSparkValue = defaultColor();
        return;
      case ALLIANCE_COLOR:
        currentSparkValue = allianceColor();
        return;
      default:
        currentSparkValue = process.getSparkValue();
        return;
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
      return allianceColor();
    }
  }

  @Override
  public void off() {
    currentSparkValue = SparkConstants.BLACK;
  }

}
