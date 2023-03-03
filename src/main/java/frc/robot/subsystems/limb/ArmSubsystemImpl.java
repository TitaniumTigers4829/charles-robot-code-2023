// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX armExtensionMotor;
  private final WPI_TalonFX beltMotor;

  private final CANCoder beltEncoder;

  @Override
  public void periodic() {}

  /** Creates a new ArmSubsystemImpl. */
  public ArmSubsystemImpl() {
    armExtensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);
    beltMotor = new WPI_TalonFX(ArmConstants.SWINGING_MOTOR_ID);

    beltEncoder = new CANCoder(ArmConstants.BELT_ENCODER_ID);
  }

  @Override
  public double getExtension() {
    double motorRotation = armExtensionMotor.getSelectedSensorPosition() * 
    (360.0 / Constants.driveFXEncoderCPR) / ArmConstants.GEAR_BOX;
    double extension = motorRotation * (ArmConstants.ARM_SPOOL_DIAMETER * Math.PI);
    return Units.inchesToMeters(extension);
  }

  @Override
  public void setExtension(double armExtension) {
    calculateFeedForward(getAngle(), getExtension());    
  }

  @Override
  public double getAngle() {
    return beltEncoder.getAbsolutePosition();
  }

  @Override
  public void goToAngle(double armRotation) {
    calculateFeedForward(getAngle(), getExtension());
  }

  @Override
  public double calculateFeedForward(double angle, double length) {
    return ( (length*ArmConstants.ARM_WEIGHT*ArmConstants.MOTOR_OHMS) / 
    ((ArmConstants.MOTOR_TORQUE/ArmConstants.MOTOR_STALL_CURRENT)*ArmConstants.GEAR_BOX) ) 
    * Math.cos(angle);
  }

}
