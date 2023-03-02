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


public class LEDSubsystemImpl extends SubsystemBase implements LEDSubsystem {

  private Spark ledSpark;

  /** Creates a new LEDSubsystemImpl.
   * @param port The SparkMax port for this LEDSubsystem.
  */
  public LEDSubsystemImpl(int port) {
    ledSpark = new Spark(port);
  }

  /** Creates a new LEDSubsystemImpl with the port in LEDConstants. */
  public LEDSubsystemImpl() {
    ledSpark = new Spark(LEDConstants.LEDPort);
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

    ledSpark.set(process.getSparkMaxValue());
  }

  private void allianceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        ledSpark.set(LEDProcess.RED_ALLIANCE.getSparkMaxValue());
    } else {
        ledSpark.set(LEDProcess.BLUE_ALLIANCE.getSparkMaxValue());
    }
  }

  private void defaultColor() {
    if (DriverStation.isAutonomous()) {
      ledSpark.set(LEDProcess.AUTONOMOUS.getSparkMaxValue());
    } else {
      allianceColor();
    }
  }

  @Override
  public void off() {
    ledSpark.set(SparkConstants.BLACK);
  }

}
