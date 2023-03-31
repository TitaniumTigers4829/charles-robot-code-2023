package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants.LEDProcess;

public interface LEDSubsystem extends Subsystem {

  /* 
    The LEDSubsystem will "remember" the current process after each time it is set.
    It holds this process until a new process is set.
    If the "quality" of the process changes, say switching from Auto to Teleop while using DEFAULT,
    the LEDSubsystem implementations may or may not update the LEDs.
    It is best practice to reset the LEDs when changes like this may occur.
  */

  /** 
   * Turns the LEDs off. 
   */
  void off();
  
  /** 
   * Sets the LEDs to a specific LEDProcess.
   * @param process the LEDProcess representing the action.
   */
  void setProcess(LEDProcess process);
  
}
