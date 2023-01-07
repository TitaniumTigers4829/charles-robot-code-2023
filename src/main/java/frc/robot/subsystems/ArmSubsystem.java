// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ClawConstants;

public class ArmSubsystem extends SubsystemBase {

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private final DoubleSolenoid solenoid;
  

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClawConstants.solenoidForward, ClawConstants.solenoidBackward);
    CloseClaw();
  }

  public void CloseClaw() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void OpenClaw() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
