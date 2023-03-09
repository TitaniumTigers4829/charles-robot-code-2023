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

  private int totalOffsetInt = 0;
  private double totalOffset = 0;
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
    // Periodic runs at 50 frames per second.
    // I hate this, but the alternative is worse.
    totalOffset += 50.0 / LEDConstants.ANIMATION_SPEED;
    totalOffsetInt = (int)totalOffset; 
    
    // Update LEDs
    setLEDColor();
  }

  @Override
  public void off() {
    currentProcess = LEDProcess.OFF;
  }

  @Override
  public void setProcess(LEDProcess process) {
    currentProcess = process;

    if (process == LEDProcess.AUTONOMOUS) {
      resetTotalOffset();
    }
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
        // slidingASCII(LEDConstants.EASTER_EGG_MESSAGE);
        // return;
        rainbowWave();
        return;
      case ALLIANCE_COLOR:
        allainceColor();
        return;
      case RED_ALLIANCE:
        animateColor(
          LEDProcess.RED_ALLIANCE.getRed(),
          LEDProcess.RED_ALLIANCE.getGreen(),
          LEDProcess.RED_ALLIANCE.getBlue(),
          LEDConstants.ALLIANCE_ANIMATION_STRENGTH
        );
        return;
      case BLUE_ALLIANCE:
        animateColor(
          LEDProcess.BLUE_ALLIANCE.getRed(),
          LEDProcess.BLUE_ALLIANCE.getGreen(),
          LEDProcess.BLUE_ALLIANCE.getBlue(),
          LEDConstants.ALLIANCE_ANIMATION_STRENGTH
        );
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

  /**
   * This method sets the color of the LEDs to the current alliance color.
   */
  private void allainceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      setProcess(LEDProcess.RED_ALLIANCE);
    } else {
      setProcess(LEDProcess.BLUE_ALLIANCE);
    }
  }

  /**
   * This rainbow effect is a wave that will display all of the colors in the spectrum and animate through them.
   */
  private void rainbowWave() {
    for (int i = 0; i < buffer.getLength(); i++) {
      final int hue = ((totalOffsetInt % 180) + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
  }

  /**
   * This rainbow effect is a flat color that cycles between all possible hues.
   */
  private void rainbow() {
    setAllLEDsHue((totalOffsetInt % 180) + (180 / buffer.getLength())  % 180);
  }

  /**
   * Slides a message across the LED strip in the form of ASCII encoding.
   * The non-static counter of totalOffsetInt must be set to zero
      if you want the first bit of the text to match the first LED.
   */
  private void slidingASCII(String text) {
    byte[] bytes = textToBytes(text);
    for (int k = 0; k < buffer.length; k++) {
      for (int i = 0; i < bytes.length; i++) {
        for (int j = 0; j < 8; j++) {
          if ((bytes[((8*i + j) + totalOffsetInt + k) % bytes.length] >> j) == 1) {
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

  /**
   * Makes a flat color look prettier by animating it slightly.
   * @param strength from 0-1, how animated the pattern is. (At zero, there is no animation, and at one, the color could be switched to any color on the entire spectrum)
   */
  private void animateColor(int red, int green, int blue, double strength) {
    int range = (int)(255 * strength);

    for (int i = 0; i < buffer.length; i++) {
      double coefficient = (2.0 * Math.PI / buffer.length);
      int relativeIndex = (i + totalOffsetInt) % buffer.length;
      buffer.setRGB(
        i,
        (red + range * Math.sin(coefficient * relativeIndex)) % 255,
        (green + range * Math.sin((Math.PI * 2.0/3.0) + (coefficient * relativeIndex))) % 255,
        (blue + range * Math.sin((Math.PI * 4.0/3.0) + (coefficient * relativeIndex))) % 255
      );
    }
  }

  /** 
   * Displays two colors to the strip in a checkerboard pattern.
   */
  private void interlaceColors(int red1, int green1, int blue1, int red2, int green2, int blue2) {
    for (int i = 0; i < buffer.length; i++) {
      if (i%2 == 0) {
        buffer.set(i, red1, green1, blue1);
      } else {
        buffer.set(i, red2, green2, blue2);
      }
    }
  }

  private void resetTotalOffset() {
    totalOffset = 0;
    totalOffsetInt = 0;
  }
}
