// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.extras.SmartDashboardLogger;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final WPI_TalonFX wristMotor;
  private final WPI_TalonFX intakeMotor;
  // private final DoubleSolenoid clawSolenoid;

  private boolean isClawClosed;
  private boolean isConeMode = true;

  public ClawSubsystemImpl() {
    wristMotor = new WPI_TalonFX(ClawConstants.WRIST_MOTOR_ID, Constants.RIO_CAN_BUS_STRING);

    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    wristMotor.config_kP(0, ClawConstants.WRIST_P);
    wristMotor.config_kI(0, ClawConstants.WRIST_I);
    wristMotor.config_kD(0, ClawConstants.WRIST_D);
    wristMotor.config_kF(0, ClawConstants.WRIST_F);

    wristMotor.config_IntegralZone(0, ClawConstants.WRIST_I_ZONE);
    wristMotor.configMotionCruiseVelocity(ClawConstants.WRIST_MAX_VELOCITY_ENCODER_UNITS);
    wristMotor.configMotionAcceleration(ClawConstants.WRIST_MAX_ACCELERATION_ENCODER_UNITS);
    wristMotor.configMotionSCurveStrength(ClawConstants.WRIST_SMOOTHING);
    wristMotor.configAllowableClosedloopError(0, ClawConstants.WRIST_TOLERANCE);

    wristMotor.configForwardSoftLimitThreshold(ClawConstants.MAX_WRIST_ROTATION_ENCODER_UNITS);
    wristMotor.configForwardSoftLimitEnable(true);
    wristMotor.configReverseSoftLimitThreshold(ClawConstants.MIN_WRIST_ROTATION_ENCODER_UNITS);
    wristMotor.configReverseSoftLimitEnable(true);
    
    wristMotor.setInverted(ClawConstants.WRIST_MOTOR_INVERTED);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setSelectedSensorPosition(0);

    intakeMotor = new WPI_TalonFX(ClawConstants.INTAKE_MOTOR_ID, Constants.RIO_CAN_BUS_STRING);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    // clawSolenoid = new DoubleSolenoid(
    //   Constants.PNEUMATICS_MODULE_TYPE,
    //   ClawConstants.SOLENOID_FORWARD,
    //   ClawConstants.SOLENOID_BACKWARD
    // );
  }

  @Override
  public void periodic() {
    SmartDashboardLogger.infoString("Cargo Mode", isConeMode ? "Cone" : "Cube");
  }

  @Override
  public void close() {
    // clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    isClawClosed = true;
  }

  @Override
  public void open() {
    // clawSolenoid.set(DoubleSolenoid.Value.kForward);    
    isClawClosed = false;
  }

  @Override
  public boolean isClawClosed() { 
    return isClawClosed;
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override 
  public void setWristMotorSpeed(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public double getWristAngle() {
    return wristMotor.getSelectedSensorPosition() * ClawConstants.WRIST_POS_TO_DEG;
  }

  @Override
  public void zeroWristEncoder() {
    wristMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void setWristPosition(double angle) {
    double angleInEncoderUnits = angle * ClawConstants.DEG_TO_WRIST_POS;
    wristMotor.set(ControlMode.MotionMagic, angleInEncoderUnits);
  }

  @Override
  public boolean isConeMode() {
    return isConeMode;
  }

  @Override
  public void switchCargoMode() {
    isConeMode = !isConeMode;
  }
  
}
