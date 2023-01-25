// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ClawConstants;

public class ArmSubsystem extends SubsystemBase {

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private final DoubleSolenoid clawSolenoid;
  private final DoubleSolenoid armSolenoid;

  public boolean clawOpen = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

     //Initialize Solenoids
     armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ArmConstants.armForward,
            ArmConstants.armBackward
    );
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ClawConstants.solenoidForward,
            ClawConstants.solenoidBackward
    );
    CloseClaw();
    CloseArm();
  }

  public void CloseClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void OpenClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void ExtendArm() {
    armSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void CloseArm() {
    armSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /** Sets the desired rotation for the wrist.
   * @param rotation The desired rotation (in radians.)
   */
  public void SetDesiredWristRotation(double rotation) {

  }


}