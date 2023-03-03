// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final DoubleSolenoid clawSolenoid;

  private final WPI_TalonFX wristMotor;

  private final CANSparkMax leftWheel;
  private final CANSparkMax rightWheel;

  private final SparkMaxPIDController leftPID;
  private final SparkMaxPIDController rightPID;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private boolean isClawClosed;

  /** Creates a new ClawSubsystemImpl. */
  public ClawSubsystemImpl() {
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ClawConstants.solenoidForward,
            ClawConstants.solenoidBackward);
    wristMotor = new WPI_TalonFX(ClawConstants.wristMotorID);

    leftWheel = new CANSparkMax(ClawConstants.leftWheelID, MotorType.kBrushless);
    rightWheel = new CANSparkMax(ClawConstants.rightWheelID, MotorType.kBrushless);

    leftWheel.restoreFactoryDefaults();
    rightWheel.restoreFactoryDefaults();

    leftWheel.setInverted(ClawConstants.leftWheelInverted);
    rightWheel.setInverted(ClawConstants.rightWheelInverted);

    leftPID = leftWheel.getPIDController();
    rightPID = rightWheel.getPIDController();
    leftEncoder = leftWheel.getEncoder();
    rightEncoder = rightWheel.getEncoder();

    leftPID.setP(ClawConstants.wheelP);
    rightPID.setP(ClawConstants.wheelP);
    leftPID.setI(ClawConstants.wheelI);
    rightPID.setI(ClawConstants.wheelI);
    leftPID.setD(ClawConstants.wheelD);
    rightPID.setD(ClawConstants.wheelD);
    leftPID.setFF(ClawConstants.wheelFF);
    rightPID.setFF(ClawConstants.wheelFF);

    leftPID.setFeedbackDevice(leftEncoder);
    rightPID.setFeedbackDevice(rightEncoder);

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
    double setpoint = speed * ClawConstants.wheelsMaxRPM;
    leftPID.setReference(setpoint, ControlType.kVelocity);
    rightPID.setReference(setpoint, ControlType.kVelocity);
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
