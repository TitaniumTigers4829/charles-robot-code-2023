// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  private final CANCoder turnEncoder;

  private final ProfiledPIDController turnPIDController =
    new ProfiledPIDController(
      ModuleConstants.moduleTurnControllerP,
      ModuleConstants.moduleTurnControllerI,
      ModuleConstants.moduleTurnControllerD,
      ModuleConstants.moduleTurnConstraints
    );

  private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
      DriveConstants.turningS, DriveConstants.turningV, DriveConstants.turningA);

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turningMotorChannel ID of the turn motor
   * @param turningEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param driveReversed is the drive motor reversed
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double angleZero,
      boolean encoderReversed,
      boolean driveReversed
      ) {
    
    // Initialize the motors
    driveMotor = new WPI_TalonFX(driveMotorChannel, ModuleConstants.canivoreCanBusString);
    turningMotor = new WPI_TalonFX(turningMotorChannel, ModuleConstants.canivoreCanBusString);

    // Set motors to brake mode
    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);

    // Handle whether motor should be reversed or not
    driveMotor.setInverted(driveReversed);
    turningMotor.setInverted(true);

    // Set the motor's built in pid and feed forward
    driveMotor.config_kF(0, ModuleConstants.moduleDriveControllerF);
    driveMotor.config_kP(0, ModuleConstants.moduleDriveControllerP);
    driveMotor.config_kI(0, ModuleConstants.moduleDriveControllerI);
    driveMotor.config_kD(0, ModuleConstants.moduleDriveControllerD);
        
    // Configure drive motor sensor
    driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    // CANCoder config
    turnEncoder = new CANCoder(turningEncoderChannel, ModuleConstants.canivoreCanBusString);
    turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    turnEncoder.configMagnetOffset(angleZero);
    turnEncoder.configSensorDirection(encoderReversed);

    // CAN stuff
    turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);

    turningMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    turningMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Code for current limiting
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
    turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
  }

  /**
   * @param currentAngle what the controller currently reads (radians)
   * @param targetAngleSetpoint the desired angle (radians)
   * @return the target angle in controller's scope (radians)
   */
  public static double calculateContinuousInputSetpoint(double currentAngle, double targetAngleSetpoint) {
    targetAngleSetpoint = Math.IEEEremainder(targetAngleSetpoint, Math.PI * 2);

    double remainder = currentAngle % (Math.PI * 2);
    double adjustedAngleSetpoint = targetAngleSetpoint + (currentAngle - remainder);

    // We don't want to rotate over 180 degrees, so just rotate the other way (add a
    // full rotation)
    if (adjustedAngleSetpoint - currentAngle > Math.PI) {
        adjustedAngleSetpoint -= Math.PI * 2;
    } else if (adjustedAngleSetpoint - currentAngle < -Math.PI) {
        adjustedAngleSetpoint += Math.PI * 2;
    }

    return adjustedAngleSetpoint;
  }

  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading(){
    return this.turnEncoder.getAbsolutePosition() % 360;
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double m_speedMetersPerSecond =
        ModuleConstants.drivetoMetersPerSecond * driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =
        (Math.PI / 180) * turnEncoder.getAbsolutePosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }

  public SwerveModulePosition getPosition() {
    double position = ModuleConstants.falconToMeters * driveMotor.getSelectedSensorPosition();
    Rotation2d rotation = Rotation2d.fromDegrees(getCANCoderABS());

    return new SwerveModulePosition(position, rotation);
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    double turnRadians = getTurnRadians();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));

    // Converts meters per second to rpm
    double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60 
      * ModuleConstants.driveGearRatio / ModuleConstants.wheelCircumferenceMeters;
      
    // Converts rpm to encoder units per 100 milliseconds
    double desiredDriveEncoderUnitsPer100MS = desiredDriveRPM / 600.0 * 2048;

    // Sets the drive motor's speed using the built in pid controller
    driveMotor.set(ControlMode.Velocity, desiredDriveEncoderUnitsPer100MS);

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput =
      turnPIDController.calculate(turnRadians, optimizedDesiredState.angle.getRadians())
        + turnFeedForward.calculate(turnPIDController.getSetpoint().velocity);
    turningMotor.set(turnOutput / 12);

    SmartDashboard.putNumber("Current Angle", turnRadians);
    SmartDashboard.putNumber("Desired Angle", optimizedDesiredState.angle.getRadians());
    SmartDashboard.putNumber("Error", optimizedDesiredState.angle.getRadians() - turnRadians);
  }

  public double getTurnRadians() {
    return ((2 * Math.PI) / 360) * turnEncoder.getAbsolutePosition();
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position
   */
  public double getCANCoderABS(){
    return turnEncoder.getAbsolutePosition();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
      turnEncoder.setPosition(0);
      driveMotor.setSelectedSensorPosition(0);
  }
}