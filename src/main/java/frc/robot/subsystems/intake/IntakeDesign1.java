// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeDesign1Constants;

public class IntakeDesign1 extends SubsystemBase {
  WPI_TalonFX rollingMotor;
  DoubleSolenoid inAndOut;

  /** Creates a new IntakeDesign1. */
  public IntakeDesign1() {
    rollingMotor = new WPI_TalonFX(IntakeDesign1Constants.ROLLING_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    rollingMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);

    inAndOut = new DoubleSolenoid(Constants.HardwareConstants.PNEUMATICS_MODULE_TYPE, IntakeDesign1Constants.FOWARD_CHANNEL, IntakeDesign1Constants.REVERSE_CHANNEL);
  }

  @Override
  public void periodic() {}

  public void extend() {
    inAndOut.set(Value.kForward);
  }

  public void retract() {
    inAndOut.set(Value.kReverse);
  }

  public void intake() {
    rollingMotor.set(IntakeDesign1Constants.ROLLING_MOTOR_INTAKE_SPEED);
  }

  public void passToArm() {
  }
}
