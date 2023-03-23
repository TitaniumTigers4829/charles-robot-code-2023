// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX leaderRotationMotor;
  private final WPI_TalonFX followerRotationMotor;

  private final CANCoder rotationEncoder;
  private final WPI_TalonFX extensionMotor;
  private final DoubleSolenoid extensionLockSolenoid;

  /** 
   * Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    leaderRotationMotor = new WPI_TalonFX(ArmConstants.LEADER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    followerRotationMotor = new WPI_TalonFX(ArmConstants.FOLLOWER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    
    rotationEncoder = new CANCoder(ArmConstants.ROTATION_ENCODER_ID, Constants.CANIVORE_CAN_BUS_STRING);
    rotationEncoder.configMagnetOffset(ArmConstants.ROTATION_ENCODER_OFFSET);
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);

    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.remoteFilter0.remoteSensorDeviceID = rotationEncoder.getDeviceID(); // must be 15 or less due to oddity in CTRE electronics
    leaderConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    leaderConfig.slot0.kP = ArmConstants.ROTATION_P;
    leaderConfig.slot0.kI = ArmConstants.ROTATION_I;
    leaderConfig.slot0.kD = ArmConstants.ROTATION_D;
    leaderConfig.slot0.closedLoopPeakOutput = 1;
    leaderConfig.motionAcceleration = ArmConstants.ROTATION_MAX_ACCELERATION * ArmConstants.ARM_DEGREES_TO_CANCODER_UNITS;
    leaderConfig.motionCruiseVelocity = ArmConstants.ROTATION_MAX_VELOCITY * ArmConstants.ARM_DEGREES_TO_CANCODER_UNITS;
    leaderConfig.motionCurveStrength = 1; // TODO: tune
    leaderRotationMotor.configAllSettings(leaderConfig);
    leaderRotationMotor.setNeutralMode(NeutralMode.Brake);
    leaderRotationMotor.setInverted(ArmConstants.LEADER_ROTATION_MOTOR_INVERTED);
    followerRotationMotor.configAllSettings(leaderConfig);
    followerRotationMotor.setNeutralMode(NeutralMode.Brake);
    followerRotationMotor.setInverted(ArmConstants.FOLLOWER_ROTATION_MOTOR_INVERTED);

    followerRotationMotor.follow(leaderRotationMotor);

    extensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.slot0.kP = ArmConstants.EXTENSION_P;
    extensionConfig.slot0.kI = ArmConstants.EXTENSION_I;
    extensionConfig.slot0.kD = ArmConstants.EXTENSION_D;
    extensionConfig.motionAcceleration = ArmConstants.EXTENSION_MAX_ACCELERATION * ArmConstants.EXTENSION_METERS_TO_MOTOR_POS;
    extensionConfig.motionCruiseVelocity = ArmConstants.EXTENSION_MAX_VELOCITY * ArmConstants.EXTENSION_METERS_TO_MOTOR_POS;
    extensionConfig.motionCurveStrength = 1; // TODO: tune
    extensionMotor.configAllSettings(extensionConfig);
    extensionMotor.setInverted(ArmConstants.EXTENSION_MOTOR_INVERTED);
    extensionMotor.setNeutralMode(NeutralMode.Coast);

    extensionMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    leaderRotationMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    followerRotationMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);

    extensionLockSolenoid = new DoubleSolenoid(
      ArmConstants.EXTENSION_LOCK_MODULE_TYPE, 
      ArmConstants.EXTENSION_LOCK_ENGAGED_ID,
      ArmConstants.EXTENSION_LOCK_DISENGAGED_ID
    );
  }

  @Override
  public void periodic() {}

  @Override
  public void setRotation(double desiredAngle) {
    double encoderPos = desiredAngle * ArmConstants.ARM_DEGREES_TO_CANCODER_UNITS;
    leaderRotationMotor.set(ControlMode.MotionMagic, encoderPos);
  }

  @Override
  public double getRotation() {
    return rotationEncoder.getAbsolutePosition();
  }

  @Override
  public void resetExtensionEncoder() {
    extensionMotor.setSelectedSensorPosition(0);
  }

  @Override
  public double getExtension() {
    return extensionMotor.getSelectedSensorPosition() * ArmConstants.EXTENSION_MOTOR_POS_TO_METERS;
  }

  @Override
  public void setExtension(double extension) {
    extensionMotor.set(ControlMode.MotionMagic, extension * ArmConstants.EXTENSION_METERS_TO_MOTOR_POS);
  }

  @Override
  public void lockExtensionSolenoid() {
    extensionLockSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void unlockExtensionSolenoid() {
    extensionLockSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public double getRotationSpeed() {
    return rotationEncoder.getVelocity();
  }

  @Override
  public void setRotationSpeed(double speed) {
    leaderRotationMotor.set(speed / 2);
  }

  @Override
  public double getExtensionSpeed() {
    // Convert motor rotation units (2048 for 1 full rotation) to number of rotations
    double motorRotation = (-extensionMotor.getSelectedSensorVelocity() / Constants.FALCON_ENCODER_RESOLUTION)
      * ArmConstants.EXTENSION_MOTOR_GEAR_RATIO;
    // Convert number of rotations to distance (multiply by diameter)
    double metersPer100MS = motorRotation * ArmConstants.EXTENSION_SPOOL_DIAMETER * Math.PI;
    return metersPer100MS * 10;
  }

  @Override
  public void setExtensionSpeed(double speed) {
    extensionMotor.set(speed);
  }

  @Override
  public double getTorqueFromGravity() {
    // Torque = mg(COM Distance*sin(theta) - r*sin(theta))
    double centerOfMassDistance = (0.4659 * getExtension()) + 0.02528; // This is the equation fit to COM distance
    double theta = Math.toRadians(getRotation() - 90); // The angle of the arm is 0 when it's pointing down
    return ArmConstants.ARM_WEIGHT_NEWTONS * 
      (centerOfMassDistance * Math.cos(theta) - ArmConstants.ARM_AXIS_OF_ROTATION_RADIUS * Math.sin(theta));
  }

  @Override
  public void setExtensionMotorNeutralMode(NeutralMode neutralMode) {
    extensionMotor.setNeutralMode(neutralMode);
  }

}
