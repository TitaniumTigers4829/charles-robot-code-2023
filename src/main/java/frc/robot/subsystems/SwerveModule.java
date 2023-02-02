// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_turnEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(
          ModuleConstants.moduleDriveControllerP,
          ModuleConstants.moduleDriveControllerI, // 0
          ModuleConstants.moduleDriveControllerD // 0
      );

  private final ProfiledPIDController m_turnPIDController =
      new ProfiledPIDController(
          ModuleConstants.moduleTurnControllerP,
          ModuleConstants.moduleTurnControllerI, // 0
          ModuleConstants.moduleTurnControllerD,
          ModuleConstants.moduleTurnConstraints);

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
      DriveConstants.voltsS, DriveConstants.voltSecondsPerMeterV, DriveConstants.voltSecondsSquaredPerMeterA);

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
    m_driveMotor = new WPI_TalonFX(driveMotorChannel, ModuleConstants.canivoreCanBusString);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel, ModuleConstants.canivoreCanBusString);

    // Set motors to brake mode
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    // Handle whether motor should be reversed or not
    m_driveMotor.setInverted(driveReversed);
    // TODO: check this
    m_turningMotor.setInverted(true);

    // Configure drive motor sensor
    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    // CANCoder config
    m_turnEncoder = new CANCoder(turningEncoderChannel, ModuleConstants.canivoreCanBusString);
    m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turnEncoder.configMagnetOffset(angleZero);
    m_turnEncoder.configSensorDirection(encoderReversed);

    // CAN stuff
    m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);

    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Code for current limiting
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
    m_turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    m_turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
  }

  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading(){
    return this.m_turnEncoder.getAbsolutePosition() % 360;
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double m_speedMetersPerSecond =
        ModuleConstants.drivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =
        (Math.PI / 180) * m_turnEncoder.getAbsolutePosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      ModuleConstants.drivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity(), 
      Rotation2d.fromDegrees(getCANCoderABS()));
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double m_speedMetersPerSecond =
        ModuleConstants.drivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turnRadians =
        ((2 * Math.PI)/360) * m_turnEncoder.getAbsolutePosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turnRadians));

    // stop the code to stop it from moving if the speed is very, very small
    if (Math.abs(state.speedMetersPerSecond) <= 0.01){
      m_turningMotor.set(0);
      m_driveMotor.set(0);
      return;
    }

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond)
             + driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turnPIDController.calculate(m_turnRadians, state.angle.getRadians())
            + turnFeedForward.calculate(m_turnPIDController.getSetpoint().velocity);
    m_driveMotor.set(driveOutput/12);
    m_turningMotor.set(turnOutput/12);
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position
   */
  public double getCANCoderABS(){
    return m_turnEncoder.getAbsolutePosition();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
      m_turnEncoder.setPosition(0);
      m_driveMotor.setSelectedSensorPosition(0);
  }
}