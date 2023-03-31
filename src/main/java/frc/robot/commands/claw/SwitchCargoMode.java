// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;

public class SwitchCargoMode extends CommandBase {

  private final ClawSubsystem clawSubsystem;
  private final LEDSubsystem leds;

  /** Creates a new SwitchCargoMode. */
  public SwitchCargoMode(ClawSubsystem clawSubsystem, LEDSubsystem leds) {
    this.clawSubsystem = clawSubsystem;
    this.leds = leds;

    addRequirements(clawSubsystem, leds);
  }

  @Override
  public void initialize() {
    clawSubsystem.switchCargoMode();
    if (clawSubsystem.isConeMode()) {
      leds.setProcess(LEDProcess.CONE);
    } else {
      leds.setProcess(LEDProcess.CUBE);
    }
  }

}
