// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.dashboard.SmartDashboardLogger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX leaderRotationMotor;
  private final WPI_TalonFX followerRotationMotor;
  private final CANCoder rotationEncoder;
  private final WPI_TalonFX extensionMotor;
  private final DoubleSolenoid extensionLockSolenoid;

  private final MotorControllerGroup rotationMotorControllerGroup;

  private final ArmFeedforward rotationFeedForward;
  private final SimpleMotorFeedforward extensionFeedForward;

  private final ProfiledPIDController rotationPIDController = new ProfiledPIDController(
    ArmConstants.ROTATION_P, 
    ArmConstants.ROTATION_I, 
    ArmConstants.ROTATION_D, 
    ArmConstants.ROTATION_CONSTRAINTS
  );
  
  private final ProfiledPIDController extensionPIDController = new ProfiledPIDController(
    ArmConstants.EXTENSION_P,
    ArmConstants.EXTENSION_I,
    ArmConstants.EXTENSION_D,
    ArmConstants.EXTENSION_CONSTRAINTS
  );

  private double currentExtensionSpeed;
  private double ticksAfterSpeedChange;

  /** Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    leaderRotationMotor = new WPI_TalonFX(ArmConstants.LEADER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    followerRotationMotor = new WPI_TalonFX(ArmConstants.FOLLOWER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);

    leaderRotationMotor.setInverted(ArmConstants.LEADER_ROTATION_MOTOR_INVERTED);
    followerRotationMotor.setInverted(ArmConstants.FOLLOWER_ROTATION_MOTOR_INVERTED);

    leaderRotationMotor.setNeutralMode(NeutralMode.Brake);
    followerRotationMotor.setNeutralMode(NeutralMode.Brake);

    rotationMotorControllerGroup = new MotorControllerGroup(leaderRotationMotor, followerRotationMotor);
    
    rotationFeedForward = new ArmFeedforward(
      ArmConstants.ROTATION_FEED_FORWARD_GAIN, 
      ArmConstants.ROTATION_VELOCITY_GAIN, 
      ArmConstants.ROTATION_ACCELERATION_GAIN
    );

    rotationEncoder = new CANCoder(ArmConstants.ROTATION_ENCODER_ID);
    rotationEncoder.configMagnetOffset(ArmConstants.EXTENSION_ENCODER_OFFSET);
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    extensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);
    extensionFeedForward = new SimpleMotorFeedforward(
      ArmConstants.EXTENSION_FEED_FORWARD_GAIN, 
      ArmConstants.EXTENSION_VELOCITY_GAIN,
      ArmConstants.EXTENSION_ACCELERATION_GAIN
    );

    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionLockSolenoid = new DoubleSolenoid(
      ArmConstants.EXTENSION_LOCK_MODULE_TYPE, 
      ArmConstants.EXTENSION_LOCK_ENGAGED_ID,
      ArmConstants.EXTENSION_LOCK_DISENGAGED_ID
    );

    currentExtensionSpeed = 0;
    ticksAfterSpeedChange = 0;
  }

  @Override
  public void goToAngle(double desiredAngle) {
    if (desiredAngle >= ArmConstants.MIN_ROTATION_DEGREES && desiredAngle <= ArmConstants.MAX_ROTATION_DEGREES) {
      double PIDOutput = rotationPIDController.calculate(getAngle(), desiredAngle);
      double feedForwardOutput = rotationFeedForward.calculate(desiredAngle, rotationPIDController.getSetpoint().velocity);
      rotationMotorControllerGroup.set(motorOutputClamp(PIDOutput + feedForwardOutput));
    }
  }

  @Override
  public double getAngle() {
    // FIXME: commented out radian conversion because goToAngle() takes in degrees
    return rotationEncoder.getAbsolutePosition();// * Math.PI / 180;
  }

  public void resetExtensionEncoder() {
    extensionMotor.setSelectedSensorPosition(0);
  }

  @Override
  public double getExtension() {
    // Convert motor rotation units (2048 or 4096 for 1 full rotation) to number of rotations
    double motorRotation = (extensionMotor.getSelectedSensorPosition() / 
      Constants.FALCON_ENCODER_RESOLUTION) * ArmConstants.EXTENSION_MOTOR_GEAR_RATIO;
    // Convert number of rotations to distance (multiply by diameter)
    double extension = motorRotation * ArmConstants.EXTENSION_SPOOL_DIAMETER * Math.PI; 
    SmartDashboardLogger.infoNumber("extension", extension);
    // Converts to output from 0 to 1
    return extension / ArmConstants.MAX_EXTENSION_LENGTH;
  }

  @Override
  public void setExtension(double desiredExtension) {
    if (desiredExtension >= ArmConstants.MIN_EXTENSION_PROPORTION && desiredExtension <= ArmConstants.MAX_EXTENSION_PROPORTION) {
      double PIDOutput = extensionPIDController.calculate(getExtension(), desiredExtension);
      double feedForwardOutput = extensionFeedForward.calculate(extensionPIDController.getSetpoint().velocity);
      extensionMotor.set(ControlMode.PercentOutput, motorOutputClamp(PIDOutput + feedForwardOutput));
      currentExtensionSpeed = motorOutputClamp(PIDOutput + feedForwardOutput);
      ticksAfterSpeedChange = 0;
    }
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
  public void setRotationSpeed(double speed) {
    rotationMotorControllerGroup.set(speed / 2);
  }

  @Override
  public double getCurrentExtensionSpeed() {
    return extensionMotor.getSelectedSensorVelocity();
  }

  @Override
  public void setCurrentExtensionSpeed(double speed) {
    extensionMotor.set(speed);
    currentExtensionSpeed = speed;
    ticksAfterSpeedChange = 0;
  }

  @Override
  public void stopArmMotorFromMoving() {
    // double speed = getCurrentExtensionSpeed();
    // double desiredSpeed = 0;
    // double motorOutput = rotationPIDController.calculate(speed, desiredSpeed);
    // extensionMotor.set(motorOutput);
  }

  @Override
  public boolean isExtensionMotorStalling() {
    return Math.abs(extensionMotor.getSelectedSensorVelocity()) < ArmConstants.STALLING_VELOCITY 
      && (Math.abs(currentExtensionSpeed) > 0) && (ticksAfterSpeedChange > ArmConstants.TICKS_BEFORE_STALL);
  }
  
  /*
   * Returns the motor output with a min. of -1 and max. of 1.
   */
  private double motorOutputClamp(double motorOutput) {
    return Math.max(-1, Math.min(1, motorOutput));
  }

  @Override
  public void periodic() {
    ticksAfterSpeedChange += 1;
    // SmartDashboardLogger.infoString("isStalling", String.valueOf(isExtensionMotorStalling()));
    // SmartDashboardLogger.infoNumber("Extension %", getExtension());
    // SmartDashboardLogger.infoNumber("Extension encoder units", extensionMotor.getSelectedSensorPosition());
    // SmartDashboardLogger.infoNumber("Rotation encoder pos:", rotationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoder pos", rotationEncoder.getAbsolutePosition());
  }
}
