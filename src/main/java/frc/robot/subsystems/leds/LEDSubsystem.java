package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants.LEDProcess;

public interface LEDSubsystem extends Subsystem {

  /** 
   * Turns the LEDs off. 
   */
  public void off();
  
  /** Sets the LEDs to a specific LEDProcess.
   * @param process the LEDProcess representing the action.
   */
  public void setProcess(LEDProcess process);
  
}
