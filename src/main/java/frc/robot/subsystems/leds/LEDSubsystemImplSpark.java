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

  /** Creates a new LEDSubsystemImpl for use with LED strips made by Spark lighting (Loopy's LEDs).
   * @param port The SparkMax port for this LEDSubsystem.
  */
  public LEDSubsystemImplSpark(int port) {
    ledSpark = new Spark(port);
    setProcess(LEDProcess.DEFAULT);
  }

  /** Creates a new LEDSubsystemImpl with the port in LEDConstants. */
  public LEDSubsystemImplSpark() {
    ledSpark = new Spark(LEDConstants.LEDPort);
    setProcess(LEDProcess.DEFAULT);
  }

  /** Sets the pattern to a double value.
   * @param value The pattern to set the LEDs to. 
   * See Constants.LEDConstants.LEDPatterns for these pattern values.
   */
  public void setSparkMax(double value) {
    ledSpark.set(value);
  }

  @Override
  public void setProcess(LEDProcess process) {
    if (process == LEDProcess.DEFAULT) {
      defaultColor();
      return;
    }
    if (process == LEDProcess.ALLIANCE_COLOR) {
      allianceColor();
      return;
    }
    if (process == LEDProcess.OFF) {
      off();
      return;
    }

    ledSpark.set(process.getSparkValue());
  }

  private void allianceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        ledSpark.set(LEDProcess.RED_ALLIANCE.getSparkValue());
    } else {
        ledSpark.set(LEDProcess.BLUE_ALLIANCE.getSparkValue());
    }
  }

  private void defaultColor() {
    if (DriverStation.isAutonomous()) {
      ledSpark.set(LEDProcess.AUTONOMOUS.getSparkValue());
    } else {
      allianceColor();
    }
  }

  @Override
  public void off() {
    ledSpark.set(SparkConstants.BLACK);
  }

}
