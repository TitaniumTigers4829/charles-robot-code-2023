// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX armExtensionMotor;
  private final SimpleMotorFeedforward armExtensionFeedForward;
  private final CANCoder armExtensionEncoder;

  private final WPI_TalonFX armRotationMotor;
  private final ArmFeedforward armRotationFeedForward;
  private final CANCoder armRotationEncoder;

  private final PIDController armRotationPIDController = new PIDController(
          ArmConstants.ROTATION_P, 
          ArmConstants.ROTATION_I, 
          ArmConstants.ROTATION_D
  );
    
  private final PIDController armExtensionPIDController = new PIDController(
          ArmConstants.EXTENSION_P,
          ArmConstants.EXTENSION_I,
          ArmConstants.EXTENSION_D
  );
  
  private final DigitalInput armExtensionLimitSwitch;
  



  /** Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    armExtensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);
    armExtensionFeedForward = new SimpleMotorFeedforward(
            ArmConstants.EXTENSION_FEED_FORWARD_GAIN, 
            ArmConstants.EXTENSION_VELOCITY_GAIN,
            ArmConstants.EXTENSION_ACCELERATION_GAIN
    );
    armExtensionEncoder = new CANCoder(ArmConstants.EXTENSION_ENCODER_ID);

    armRotationMotor = new WPI_TalonFX(ArmConstants.ROTATION_MOTOR_ID);
    armRotationFeedForward = new ArmFeedforward(
            ArmConstants.ROTATION_FEED_FORWARD_GAIN, 
            ArmConstants.ROTATION_VELOCITY_GAIN, 
            ArmConstants.ROTATION_ACCELERATION_GAIN
    );
    armRotationEncoder = new CANCoder(ArmConstants.ROTATION_ENCODER_ID);

    armExtensionLimitSwitch = new DigitalInput(ArmConstants.EXTENSION_LIMIT_SWITCH_ID);

    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() { }

  @Override
  public void goToAngle(double desiredAngle, double currentAngle, double currentVelocity) {
    double feedForwardOutput = armRotationFeedForward.calculate(desiredAngle, currentVelocity);
    double PIDOutput = armRotationPIDController.calculate(currentAngle, desiredAngle);
    double motorOutput = (PIDOutput + feedForwardOutput);
    armRotationMotor.set(ControlMode.PercentOutput, motorOutput);
  }

  @Override
  public double getAngle() {
    return armRotationEncoder.getAbsolutePosition() * Math.PI / 180;
  }

  public double getCurrentRotationVelocity() {
    return armRotationEncoder.getVelocity();
  }

  @Override
  public double getExtension() {
    return armExtensionEncoder.getPosition();
  }

  public double getExtensionVelocity() {
    return armExtensionEncoder.getVelocity();
  }
  
  public void resetExtensionEncoder() {
    boolean extensionLimitSwitchPressed = armExtensionLimitSwitch.get();
    if (extensionLimitSwitchPressed) {
      armExtensionEncoder.setPosition(0);
    }
  }
  /** 
   * Sets the arm's extension from 0 to 1.
   */
  @Override
  public void setExtension(double desiredAngle, double extensionAngle, double extensionVelocity) {
    
    double feedForwardOutput = armExtensionFeedForward.calculate(extensionVelocity);
    double PIDOutput = armExtensionPIDController.calculate(extensionAngle, desiredAngle);
    double motorOutput = (PIDOutput + feedForwardOutput);

    if (motorOutput > 1) {
      armExtensionMotor.set(ControlMode.PercentOutput, 1);  
    }
    else {
      armExtensionMotor.set(ControlMode.PercentOutput, motorOutput);
    }
  }


}