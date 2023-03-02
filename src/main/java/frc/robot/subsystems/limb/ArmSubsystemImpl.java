// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX armExtensionMotor;
  private final WPI_TalonFX beltMotor;
  private final ArmFeedforward armFeedForward;
  private final CANCoder armEncoder;
  private final PIDController armPIDController =
  new PIDController(
      ArmConstants.pValue,
      ArmConstants.iValue, // 0
      ArmConstants.dValue
  );
  
  



  /** Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    armExtensionMotor = new WPI_TalonFX(ArmConstants.extensionMotorID);
    beltMotor = new WPI_TalonFX(ArmConstants.swingingMotorID);
    armFeedForward = new ArmFeedforward(ArmConstants.feedForwardGain, 
    ArmConstants.velocityGain, ArmConstants.accelerationGain);
    armEncoder = new CANCoder(ArmConstants.armEncoderID);
  }

  @Override
  public void periodic() { }

  @Override
  public void goToAngle(double desiredAngle, double currentAngle, double currentVelocity) {
    double feedForwardOutput = armFeedForward.calculate(desiredAngle, currentVelocity);
    double PIDOutput = armPIDController.calculate(currentAngle, desiredAngle);
    double motorOutput = (PIDOutput + feedForwardOutput);
    beltMotor.set(ControlMode.PercentOutput, motorOutput);
  }

  @Override
  public double getAngle() {
    double currentAngle = armEncoder.getAbsolutePosition();
    return currentAngle;
  }

  public double getCurrentArmVelocity() {
    double currentVelocity = armEncoder.getVelocity();
    return currentVelocity;
  }

  @Override
  public boolean getExtension() {
    // TODO Auto-generated method stub
    return false;
  }
  
  /** 
   * Sets the arm's extension from 0 to 1.
   */
  @Override
  public void setExtension(double armExtension) {
    
  }


}