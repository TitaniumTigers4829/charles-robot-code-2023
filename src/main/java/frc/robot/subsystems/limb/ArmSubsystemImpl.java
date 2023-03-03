// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limb;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_TalonFX armExtensionMotor;
  private final WPI_TalonFX beltMotor;

  @Override
  public void periodic() {}

  /** Creates a new ArmSubsystemImpl. */
  public ArmSubsystemImpl() {
    armExtensionMotor = new WPI_TalonFX(ArmConstants.extensionMotorID);
    beltMotor = new WPI_TalonFX(ArmConstants.swingingMotorID);
  }

  @Override
  public boolean getExtension() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setExtension(double armExtension) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getAngle() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void goToAngle(double armRotation) {
    // TODO Auto-generated method stub
  }

}
