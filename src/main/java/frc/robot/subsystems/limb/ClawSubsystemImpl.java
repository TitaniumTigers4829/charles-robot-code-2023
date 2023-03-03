// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final DoubleSolenoid clawSolenoid;

  private final WPI_TalonFX wristMotor;
  private final WPI_TalonSRX leftClawMotor;
  private final WPI_TalonSRX rightClawMotor;

  private boolean isClawClosed;

  /** Creates a new ClawSubsystemImpl. */
  public ClawSubsystemImpl() {
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ClawConstants.solenoidForward,
            ClawConstants.solenoidBackward);
    wristMotor = new WPI_TalonFX(ClawConstants.wristMotorID);
    leftClawMotor = new WPI_TalonSRX(ClawConstants.leftClawMotorID);
    rightClawMotor = new WPI_TalonSRX(ClawConstants.rightClawMotorID);
  }

  @Override
  public void periodic() {}

  @Override
  public void close() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void open() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public boolean getClawClosed() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setMotorSpeed(double speed) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getWristAngle() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void setWristAngle(double angle) {
    // TODO Auto-generated method stub
    
  }


}
