// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import java.util.ResourceBundle.Control;

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
            ClawConstants.SOLENOID_FORWARD,
            ClawConstants.SOLENOID_BACKWARD);
    wristMotor = new WPI_TalonFX(ClawConstants.WRIST_MOTOR_ID);

    leftWheel = new CANSparkMax(ClawConstants.LEFT_WHEEL_MOTOR_ID, MotorType.kBrushless);
    rightWheel = new CANSparkMax(ClawConstants.RIGHT_WHEEL_MOTOR_ID, MotorType.kBrushless);

    leftWheel.restoreFactoryDefaults();
    rightWheel.restoreFactoryDefaults();

    leftWheel.setInverted(ClawConstants.LEFT_WHEEL_MOTOR_INVERTED);
    rightWheel.setInverted(ClawConstants.RIGHT_WHEEL_MOTOR_INVERTED);

    leftPID = leftWheel.getPIDController();
    rightPID = rightWheel.getPIDController();
    leftEncoder = leftWheel.getEncoder();
    rightEncoder = rightWheel.getEncoder();

    leftPID.setP(ClawConstants.WHEEL_P);
    rightPID.setP(ClawConstants.WHEEL_P);
    leftPID.setI(ClawConstants.WHEEL_I);
    rightPID.setI(ClawConstants.WHEEL_I);
    leftPID.setD(ClawConstants.WHEEL_D);
    rightPID.setD(ClawConstants.WHEEL_D);
    leftPID.setFF(ClawConstants.WHEEL_FEED_FORWARD_GAIN);
    rightPID.setFF(ClawConstants.WHEEL_FEED_FORWARD_GAIN);

    leftPID.setFeedbackDevice(leftEncoder);
    rightPID.setFeedbackDevice(rightEncoder);

  }

  @Override
  public void periodic() {}

  @Override
  public void close() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    isClawClosed = true;    
  }

  @Override
  public void open() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse); 
    isClawClosed = false;   
  }

  @Override
  public boolean getClawClosed() { 
    return isClawClosed;
  }

  @Override
  public void setMotorSpeed(double speed) {
    double setpoint = speed * ClawConstants.WHEELS_MAX_RPM;
    leftPID.setReference(setpoint, ControlType.kVelocity);
    rightPID.setReference(setpoint, ControlType.kVelocity);
  }

  @Override
  public double getWristAngle() {
    return wristMotor.getSelectedSensorPosition() * (360 / 2048.);
  }

  @Override
  public void setWristAngle(double angle) {
    double desiredPos = angle * (2048/360.);
    wristMotor.set(ControlMode.MotionMagic, desiredPos);
  }

}
