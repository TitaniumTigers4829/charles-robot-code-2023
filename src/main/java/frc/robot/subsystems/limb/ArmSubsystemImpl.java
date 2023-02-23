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
  public void extendArm() {
//TODO: Stub
  }

  @Override
  public void retractArm() {
//TODO: Stub
  }

  @Override
  public void toggleArm() {
//TODO: Stub
  }

  @Override
  public boolean getArmExtended() {
    return false; //TODO: Stub
  }

  @Override
  public double getAngle() {
    return 0; //TODO: Stub
  }

  @Override
  public void goToAngle(double angle) {
    //TODO: Stub
  }
}
