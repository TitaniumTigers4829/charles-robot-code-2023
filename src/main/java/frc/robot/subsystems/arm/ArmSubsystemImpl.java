// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_CANCoder rotationEncoder;
  private final WPI_TalonFX leaderRotationMotor;
  private final WPI_TalonFX followerRotationMotor;
  private final WPI_TalonFX extensionMotor;
  private final DoubleSolenoid extensionLockSolenoid;

  private final ProfiledPIDController extensionSpeedPIDController = new ProfiledPIDController(
    ArmConstants.EXTENSION_P,
    ArmConstants.EXTENSION_I,
    ArmConstants.EXTENSION_D,
    ArmConstants.EXTENSION_CONSTRAINTS
  );

  /** 
   * Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    rotationEncoder = new WPI_CANCoder(ArmConstants.ROTATION_ENCODER_ID, Constants.CANIVORE_CAN_BUS_STRING);
    leaderRotationMotor = new WPI_TalonFX(ArmConstants.LEADER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    followerRotationMotor = new WPI_TalonFX(ArmConstants.FOLLOWER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    extensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);
    extensionLockSolenoid = new DoubleSolenoid(ArmConstants.EXTENSION_LOCK_MODULE_TYPE, ArmConstants.EXTENSION_LOCK_ENGAGED_ID, ArmConstants.EXTENSION_LOCK_DISENGAGED_ID);
    
    rotationEncoder.configMagnetOffset(ArmConstants.ROTATION_ENCODER_OFFSET);
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
    rotationEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);

    // leaderRotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0, 0);
    // leaderRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    // leaderRotationMotor.config_kP(0, ArmConstants.ROTATION_P);
    // leaderRotationMotor.config_kI(0, ArmConstants.ROTATION_I);
    // leaderRotationMotor.config_kD(0, ArmConstants.ROTATION_D);
    // leaderRotationMotor.configMotionCruiseVelocity(ArmConstants.ROTATION_MAX_VELOCITY_ENCODER_UNITS);
    // leaderRotationMotor.configMotionAcceleration(ArmConstants.ROTATION_MAX_ACCELERATION_ENCODER_UNITS);
    // leaderRotationMotor.configMotionSCurveStrength(ArmConstants.ROTATION_SMOOTHING);
    // leaderRotationMotor.configAllowableClosedloopError(0, ArmConstants.ROTATION_TOLERANCE);
    // leaderRotationMotor.configForwardSoftLimitThreshold(ArmConstants.MAX_ROTATION_ENCODER_UNITS);
    // leaderRotationMotor.configForwardSoftLimitEnable(true);
    // leaderRotationMotor.configReverseSoftLimitThreshold(ArmConstants.MIN_ROTATION_ENCODER_UNITS);
    // leaderRotationMotor.configReverseSoftLimitEnable(true);

    // leaderRotationMotor.setNeutralMode(NeutralMode.Brake);

    // // followerRotationMotor.setInverted(ArmConstants.FOLLOWER_ROTATION_MOTOR_INVERTED);
    // followerRotationMotor.setInverted(InvertType.OpposeMaster);
    // followerRotationMotor.setNeutralMode(NeutralMode.Brake);

    // followerRotationMotor.setSensorPhase(true);
    
    // followerRotationMotor.follow(leaderRotationMotor);

    extensionMotor.setInverted(ArmConstants.EXTENSION_MOTOR_INVERTED);
    extensionMotor.setNeutralMode(NeutralMode.Coast);

    extensionMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // leaderRotationMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // followerRotationMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    leaderRotationMotor.setSelectedSensorPosition(0);
    followerRotationMotor.setSelectedSensorPosition(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    config.remoteFilter0.remoteSensorDeviceID = rotationEncoder.getDeviceID();
    // config.diff0Term = FeedbackDevice.RemoteSensor0;
    // config.sum0Term = FeedbackDevice.RemoteSensor0;

    config.slot0.kP = ArmConstants.ROTATION_P;
    config.slot0.kI = ArmConstants.ROTATION_I;
    config.slot0.kD = ArmConstants.ROTATION_D;

    config.motionCruiseVelocity = ArmConstants.ROTATION_MAX_VELOCITY_ENCODER_UNITS;
    config.motionAcceleration = ArmConstants.ROTATION_MAX_ACCELERATION_ENCODER_UNITS;
    config.motionCurveStrength = ArmConstants.ROTATION_SMOOTHING;

    config.slot0.allowableClosedloopError = ArmConstants.ROTATION_TOLERANCE;

    config.forwardSoftLimitThreshold = ArmConstants.MAX_ROTATION_ENCODER_UNITS;
    config.reverseSoftLimitThreshold = ArmConstants.MIN_ROTATION_ENCODER_UNITS;
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;

    leaderRotationMotor.configAllSettings(config);
    followerRotationMotor.configAllSettings(config);

    leaderRotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0, 0);
    leaderRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    followerRotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0, 0);
    followerRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);

    followerRotationMotor.setSensorPhase(true);

    leaderRotationMotor.setNeutralMode(NeutralMode.Brake);
    followerRotationMotor.setNeutralMode(NeutralMode.Brake);

    followerRotationMotor.setInverted(InvertType.OpposeMaster);

    followerRotationMotor.follow(leaderRotationMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("cancoder pos", rotationEncoder.getAbsolutePosition() * Constants.DEGREES_TO_CANCODER_UNITS);
    SmartDashboard.putNumber("leader pos", leaderRotationMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("follower pos", followerRotationMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("leader out", leaderRotationMotor.get());
    SmartDashboard.putNumber("follower out", followerRotationMotor.get());
  }

  @Override
  public void setRotation(double desiredAngle) {
    leaderRotationMotor.set(ControlMode.MotionMagic, desiredAngle * Constants.DEGREES_TO_CANCODER_UNITS);
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
    // Convert motor rotation units (2048 or 4096 for 1 full rotation) to number of rotations
    double motorRotation = (-extensionMotor.getSelectedSensorPosition() / 
      Constants.FALCON_ENCODER_RESOLUTION) * ArmConstants.EXTENSION_MOTOR_GEAR_RATIO;
    // Convert number of rotations to distance (multiply by diameter)
    return motorRotation * ArmConstants.EXTENSION_SPOOL_DIAMETER * Math.PI;
  }

  @Override
  public void setExtension(double extension) {
    // FIXME: rework so that this "just works" and the "user" doesn't need to worry about pneumatic brake or anything else.
    // FIXME: we should also make this "safe" against going outside of frame perimeter, don't allow the user to make the arm go outside frame while this is running
    // EXAMPLE IS FOR BOTH SET EXTENSION AND SET ROTATION
    // example: if the arm is mostly extended and we want the arm to go from 90 to 270 (or however the degrees work i forget), but basically a 180 over the top,
    // this method will handle pulling the arm in while it is in motion, and then return the arm to the desired extension.
    // I have some ideas of how to do this, a cool way to visualize it is something called configuration space, see discord.
    // Another thing that we need protect against is rotation and extension values that our not in our configuration space.
    double PIDOutput = extensionSpeedPIDController.calculate(getExtension(), extension);
    // Sets a floor
    PIDOutput = Math.max(PIDOutput, ArmConstants.EXTENSION_MOTOR_MIN_OUTPUT);
    // Sets a ceiling
    PIDOutput = Math.min(PIDOutput, ArmConstants.EXTENSION_MOTOR_MAX_OUTPUT);
    setExtensionSpeed(PIDOutput);
  }

  public void syncRotationEncoders() {
    leaderRotationMotor.setSelectedSensorPosition(getRotation() * ArmConstants.ARM_DEGREES_TO_FALCON_UNITS);
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

  @Override
  public void resetExtensionController() {
    extensionSpeedPIDController.reset(getExtension(), getExtensionSpeed());
  }

}
