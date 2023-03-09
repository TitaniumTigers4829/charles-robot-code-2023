// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDProcess;

public class LEDSubsystemImplBTF extends SubsystemBase implements LEDSubsystem {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  private int rainbowOffset = 0;
  private int totalOffset = 0;
  private LEDProcess currentProcess = LEDProcess.OFF;

  /** Creates a new LEDSubsystemImpl for use with the WS2812b Adressable LED strips. 
   * @param port the PWM port of the LED strips
   * @param length the amount of LEDs in the total strip
  */
  public LEDSubsystemImplBTF(int port, int length) {
    led = new AddressableLED(port);
    led.setLength(length);
    buffer = new AddressableLEDBuffer(length);
    led.setData(buffer);
    led.start();
  }

  @Override
  public void periodic() {  
    // Update LEDs
    setLEDColor();

    // Update all the looping offsets
    rainbowOffset += 3;
    rainbowOffset %= 180;

    totalOffset++;
  }

  @Override
  public void off() {
    currentProcess = LEDProcess.OFF;
  }

  @Override
  public void setProcess(LEDProcess process) {
    currentProcess = process;
  }

  private void setLEDColor() {
    switch (currentProcess) {
      case DEFAULT:
        defaultColor();
        return;
      case RAINBOW:
        rainbow();
        return;
      case AUTONOMOUS:
        rainbowWave();
        return;
      case ALLIANCE_COLOR:
        allainceColor();
        return;
      default:
        setAllLEDs(currentProcess.getRed(), currentProcess.getGreen(), currentProcess.getBlue());
        return;
    }
  }

  private void setAllLEDs(int red, int green, int blue) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, red, green, blue);
    }
  }

  private void setAllLEDsHue(int hue) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, hue, 255, 128);
    }
  }

  private void defaultColor() {
    if (DriverStation.isAutonomous()) {
      setProcess(LEDProcess.AUTONOMOUS);
    } else {
      setProcess(LEDProcess.ALLIANCE_COLOR);
    }
  }

  private void allainceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      setProcess(LEDProcess.RED_ALLIANCE);
    } else {
      setProcess(LEDProcess.BLUE_ALLIANCE);
    }
  }

  private void rainbowWave() {
    for (int i = 0; i < buffer.getLength(); i++) {
      final int hue = (rainbowOffset + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
  }

  private void rainbow() {
    setAllLEDsHue(rainbowOffset + (180 / buffer.getLength())  % 180);
  }

  /**
   * Slides a message across the LED strip in the form of ASCII encoding.
   * The non-static counter of totalOffset must be set to zero
      if you want the first bit of the text to match the first LED.
   */
  private void slidingASCII(String text) {
    byte[] bytes = textToBytes(text);
    for (int k = 0; k < buffer.length; k++) {
      for (int i = 0; i < bytes.length; i++) {
        for (int j = 0; j < 8; j++) {
          if ((bytes[((8*i + j) + totalOffset + k) % bytes.length] >> j) == 1) {
            buffer.setRGB(k, 255, 255, 255);
          } else {
            buffer.setRGB(k, 0, 0, 0);
          }
        }
      }
    }
  }

  /**
   * Returns the ASCII encoding of the input string.
   */
  private byte[] textToBytes(String text) {
    return text.getBytes(StandardCharsets.US_ASCII);
  }
}
