// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDPatterns;

public class LEDSubsystem extends SubsystemBase {

  private Spark ledSpark;

  /** Creates a new LEDSubsystem. 
   * @param port The SparkMax port for this LEDSubsystem.
  */
  public LEDSubsystem(int port) {
    ledSpark = new Spark(port);
  }

  /** Creates a new LEDSubsystem with the port in LEDConstants. */
  public LEDSubsystem() {
    ledSpark = new Spark(LEDConstants.LEDPort);
  }

  /** Sets the pattern to a double value.
   * @param value The pattern to set the LEDs to. 
   * See Constants.LEDConstants.LEDPatterns for these pattern values.
   */
  public void setLEDColor(double value) {
    ledSpark.set(value);
  }

  /** Set the LED color to the current alliance color. */
  public void setLEDAllianceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        ledSpark.set(LEDPatterns.FIRE);
    } else {
        ledSpark.set(LEDPatterns.OCEAN);
    }
  }

  /** Sets the LED to its default value.
   * This can be changed if needed.
   */
  public void setLEDDefault() {
    if (DriverStation.isAutonomous()) {
      setLEDAuto();
    } else {
      setLEDAllianceColor();
    }
  }

  public void setLEDScoring() {
    ledSpark.set(LEDPatterns.YELLOW);
  }
  
  public void setLEDLiningUp() {
    ledSpark.set(LEDPatterns.WHITE);
  }

  public void setLEDBalancing() {
    ledSpark.set(LEDPatterns.CYAN);
  }

  public void setLEDIntake() {
    ledSpark.set(LEDPatterns.MAGENTA);
  }

  public void setLEDAuto() {
    ledSpark.set(LEDPatterns.RAINBOW);
  }
  
  public void setLEDTest() {
    ledSpark.set(LEDPatterns.RAINBOW);
  }

  //TODO: Create more led methods for different situations.

}
