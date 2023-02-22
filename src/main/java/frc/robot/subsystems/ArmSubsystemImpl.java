// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ClawConstants;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private final DoubleSolenoid clawSolenoid;
  private final DoubleSolenoid armSolenoid;

  private boolean clawOpen = false;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystemImpl() {

     //Initialize Solenoids
     armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ArmConstants.armForward,
            ArmConstants.armBackward
    );
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ClawConstants.solenoidForward,
            ClawConstants.solenoidBackward
    );
    closeClaw();
    closeArm();
  }

  public void invertClaw() {
    clawOpen = !clawOpen;
  }

  public boolean getClaw() {
    return clawOpen;
  }

  @Override
  public void closeClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void openClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void extendArm() {
    armSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void closeArm() {
    armSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  @Override
  public void setDesiredWristRotation(double rotation) {

  }
}
