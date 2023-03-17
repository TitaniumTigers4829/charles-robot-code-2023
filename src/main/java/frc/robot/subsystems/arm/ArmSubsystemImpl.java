// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.dashboard.SmartDashboardLogger;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

  private final ProfiledPIDController rotationPIDController = new ProfiledPIDController(
    ArmConstants.ROTATION_P, 
    ArmConstants.ROTATION_I, 
    ArmConstants.ROTATION_D, 
    ArmConstants.ROTATION_CONSTRAINTS
  );
  
  private final ProfiledPIDController extensionSpeedPIDController = new ProfiledPIDController(
    ArmConstants.EXTENSION_P,
    ArmConstants.EXTENSION_I,
    ArmConstants.EXTENSION_D,
    ArmConstants.EXTENSION_CONSTRAINTS
  );

  private double consecutiveHighAmpLoops = 0;
  private String cargoMode = "Cone";

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
    
    rotationEncoder = new CANCoder(ArmConstants.ROTATION_ENCODER_ID);
    rotationEncoder.configMagnetOffset(ArmConstants.EXTENSION_ENCODER_OFFSET);
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    extensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);

    extensionMotor.setInverted(ArmConstants.EXTENSION_MOTOR_INVERTED);
    extensionMotor.setNeutralMode(NeutralMode.Coast);

    extensionLockSolenoid = new DoubleSolenoid(
      ArmConstants.EXTENSION_LOCK_MODULE_TYPE, 
      ArmConstants.EXTENSION_LOCK_ENGAGED_ID,
      ArmConstants.EXTENSION_LOCK_DISENGAGED_ID
    );
  }

  @Override
  public void setRotation(double desiredAngle) {
    double PIDOutput = rotationPIDController.calculate(getRotation(), desiredAngle);
    double feedForwardOutput = ArmConstants.ROTATION_FEED_FORWARD_CONSTANT * getTorqueFromGravity();
    SmartDashboard.putNumber("Rot PID Output", PIDOutput);
    setRotationSpeed(PIDOutput + feedForwardOutput);
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
    double PIDOutput = extensionSpeedPIDController.calculate(getExtension(), extension);
    // Sets a floor
    PIDOutput = Math.max(PIDOutput, ArmConstants.EXTENSION_MOTOR_MIN_OUTPUT);
    // Sets a ceiling
    PIDOutput = Math.min(PIDOutput, ArmConstants.EXTENSION_MOTOR_MAX_OUTPUT);
    setExtensionSpeed(PIDOutput);
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
    rotationMotorControllerGroup.set(speed / 2);
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
  public boolean isExtensionMotorStalling() {
    return consecutiveHighAmpLoops >= 20;
  }

  @Override
  public void setExtensionMotorNeutralMode(NeutralMode neutralMode) {
    extensionMotor.setNeutralMode(neutralMode);
  }
  
  @Override
  public void resetRotationController() {
    rotationPIDController.reset(getRotation(), getExtensionSpeed());
  }

  @Override
  public void resetExtensionController() {
    extensionSpeedPIDController.reset(getExtension(), getExtensionSpeed());
  }

  @Override
  public void switchCargoMode() {
    if (cargoMode == "Cone") {
      cargoMode = "Cube";
    } else {
      cargoMode = "Cone";
    }
  }

  @Override
  public String getCargoMode() {
    return cargoMode;
  }

  @Override
  public void periodic() {
    SmartDashboardLogger.infoNumber("extension (meters)", getExtension());
    SmartDashboardLogger.infoNumber("encoder pos", rotationEncoder.getAbsolutePosition());
    SmartDashboardLogger.infoNumber("arm vel", getRotationSpeed());
    SmartDashboardLogger.infoString("Cargo Mode", cargoMode);

    if (extensionMotor.getSupplyCurrent() > ArmConstants.EXTENSION_MOTOR_STALLING_AMPS) {
      consecutiveHighAmpLoops++;
    } else {
      consecutiveHighAmpLoops = 0;
    }
  }

  /*
   * Returns the motor output with a min. of -1 and max. of 1.
   */
  private double motorOutputClamp(double motorOutput) {
    return Math.max(-1, Math.min(1, motorOutput));
  }

}
