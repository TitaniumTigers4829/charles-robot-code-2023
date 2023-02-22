package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants.LEDProcess;

public interface LEDSubsystem extends Subsystem {
    /** Turns the LEDs off. */
    void off();
    /** Sets the LEDs to a specific LEDProcess.
     * @param process the LEDProcess representing the action.
     */
    void setProcess(LEDProcess process);
}
