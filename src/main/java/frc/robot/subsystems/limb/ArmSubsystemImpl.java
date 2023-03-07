// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX rotationMotor;
  private final CANCoder rotationEncoder;
  private final WPI_TalonFX extensionMotor;

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

  /** Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    rotationMotor = new WPI_TalonFX(ArmConstants.ROTATION_MOTOR_ID);
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
  }

  @Override
  public void periodic() { }

  @Override
  public void goToAngle(double desiredAngle) {
    double PIDOutput = rotationPIDController.calculate(getAngle(), desiredAngle);
    double feedForwardOutput = rotationFeedForward.calculate(desiredAngle, rotationPIDController.getSetpoint().velocity);
    // Makes sure it doesn't set the percent output to more than 100%
    rotationMotor.set(ControlMode.PercentOutput, motorOutputClamp(PIDOutput + feedForwardOutput));
  }

  @Override
  public double getAngle() {
    return rotationEncoder.getAbsolutePosition() * Math.PI / 180;
  }

  @Override
  public double getExtension() {
    // Convert motor rotation units (2048 or 4096 for 1 full rotation) to number of rotations
    double motorRotation = extensionMotor.getSelectedSensorPosition() * 
      (360 / Constants.FALCON_ENCODER_RESOLUTION) / ArmConstants.EXTENSION_MOTOR_GEAR_RATIO;
    // Convert number of rotations to distance (multiply by diamter)
    double extension = motorRotation * ArmConstants.EXTENSION_SPOOL_DIAMETER * Math.PI; 
    // Converts to output from 0 to 1
    return extension / ArmConstants.MAX_EXTENSION_LENGTH;
  }

  @Override
  public void setExtension(double desiredExtension) {
    double PIDOutput = extensionPIDController.calculate(getExtension(), desiredExtension);
    double feedForwardOutput = extensionFeedForward.calculate(extensionPIDController.getSetpoint().velocity);
    extensionMotor.set(ControlMode.PercentOutput, motorOutputClamp(PIDOutput + feedForwardOutput));
  }

  /*
   * Returns the motor output with a min. of -1 and max. of 1.
   */
  private double motorOutputClamp(double motorOutput) {
    return Math.max(-1, Math.min(1, motorOutput));
  }

}