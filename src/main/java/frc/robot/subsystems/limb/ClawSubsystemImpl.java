// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final DoubleSolenoid clawSolenoid;

  private final WPI_TalonFX wristMotor;

  private final CANSparkMax leftWheel;
  private final CANSparkMax rightWheel;


  private boolean isClawClosed;

  private final DigitalInput wristLimitSwitch;

  // private final SimpleMotorFeedforward wristFeedForward;
  // private final ProfiledPIDController wristPIDController;

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

    wristLimitSwitch = new DigitalInput(ClawConstants.wristLimitSwitchPort);

    // wristFeedForward = new SimpleMotorFeedforward(
    //     ClawConstants.WRIST_FEED_FORWARD_GAIN, 
    //     ClawConstants.WRIST_VELOCITY_GAIN, 
    //     ClawConstants.WRIST_ACCELERATION_GAIN
    // );

    // wristPIDController = new ProfiledPIDController
    //         (ClawConstants.WRIST_P, ClawConstants.WRIST_I, ClawConstants.WRIST_D, ClawConstants.WRIST_CONSTRAINTS);

    wristMotor.config_kF(0, ClawConstants.WRIST_FEED_FORWARD_GAIN, 0);
    wristMotor.config_kP(0, ClawConstants.WRIST_P, 0);
    wristMotor.config_kI(0, ClawConstants.WRIST_I, 0);
    wristMotor.config_IntegralZone(0, 150.0 / (600.0) * Constants.FALCON_ENCODER_RESOLUTION); //This ratio was taken from climb subsystem, I don't know if it needs changing

    wristMotor.setNeutralMode(NeutralMode.Brake);
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
    double motorOutput = speed * ClawConstants.WHEELS_MAX_RPM;
    leftWheel.set(motorOutputClamp(motorOutput));
    rightWheel.set(motorOutputClamp(motorOutput));
  }

  /*
   * Returns the motor output with a min. of -1 and max. of 1
   */
  public double motorOutputClamp(double motorOutput) {
    return Math.max(-1, Math.min(1, motorOutput));
  }

  @Override
  public double getWristAngle() {
    return wristMotor.getSelectedSensorPosition() * (360 / Constants.FALCON_ENCODER_RESOLUTION);
  }

  public boolean isLmitSwitchPressed() {
    return wristLimitSwitch.get();
  }

  public double resetWristEncoder() {
    if (isLmitSwitchPressed()) {
      return ClawConstants.MIN_WRIST_ROTATION;
    }
    else {
      return getWristAngle();
    }
  }

  @Override
  public void setWristAngle(double desiredAngle) {
    double desiredPos = desiredAngle * (Constants.FALCON_ENCODER_RESOLUTION/360);

    wristMotor.set(ControlMode.MotionMagic, desiredPos);

    // double PIDOutput = wristPIDController.calculate(getWristAngle(), desiredAngle);
    // double feedForwardOutput = wristFeedForward.calculate(desiredAngle, wristPIDController.getSetpoint().velocity);
    // wristMotor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, PIDOutput + feedForwardOutput)));
  }

}
