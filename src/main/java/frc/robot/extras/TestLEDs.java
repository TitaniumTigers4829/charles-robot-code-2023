// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extras;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.subsystems.leds.LEDSubsystem;

public class TestLEDs extends CommandBase {

  private final LEDSubsystem leds;

  /** Creates a new TestLEDs. */
  public TestLEDs(LEDSubsystem leds) {
    this.leds = leds;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.setProcess(LEDProcess.SCORING_CUBE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.SCORING_CONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
