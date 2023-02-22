// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // The claw should be grabbing when the match starts.
    grab();
  }

  @Override
  public void periodic() {}

  @Override
  public void grab() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    isClawClosed = true;
  }

  @Override
  public void release() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    isClawClosed = false;
  }

  @Override
  public boolean getClawClosed() {
    return isClawClosed;
  }

  @Override
  public void stopMotors() {
    spinMotors(0, 0);
  }

  @Override
  public void setWristAngle(double angle) {
//TODO: Stub
  }

  @Override
  public double getWristAngle() {
    return -1;//TODO: Stub
  }

  @Override
  public void toggleClaw() {
    if (isClawClosed) {
      release();
    } else {
      grab();
    }
  }

  @Override
  public void spinMotors(double leftSpeed, double rightSpeed) {
  leftClawMotor.set(leftSpeed);
  rightClawMotor.set(rightSpeed);
  }

  @Override
  public void spinMotors(double speed) {
    spinMotors(speed, speed);
  }

}
