// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeDesign2Constants;
import frc.robot.extras.NodeAndModeRegistry;

public class IntakeDesign2 extends SubsystemBase {
  WPI_TalonFX leftMotor, rightMotor;

  /** Creates a new IntakeDesign2. */
  public IntakeDesign2() {
    leftMotor = new WPI_TalonFX(IntakeDesign2Constants.INTAKE_MOTOR_LEFT_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    rightMotor = new WPI_TalonFX(IntakeDesign2Constants.INTAKE_MOTOR_RIGHT_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);

    leftMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    rightMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    leftMotor.setInverted(IntakeDesign2Constants.MOTOR_LEFT_INVERTED);
    rightMotor.setInverted(IntakeDesign2Constants.MOTOR_RIGHT_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    // TODO: figure out method for extending intake
    leftMotor.set(IntakeDesign2Constants.SUCC_SPEED);
    rightMotor.set(IntakeDesign2Constants.SUCC_SPEED);
  }

  public void manual(double leftSpeed, double rightSpeed) {
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void getRidOfCargo() {
    // TODO: rotate

    int node = NodeAndModeRegistry.getSelectedNode();
    if (node >= 19) {
      leftMotor.set(IntakeDesign2Constants.HIGH_SHOOT_SPEED);
      rightMotor.set(IntakeDesign2Constants.HIGH_SHOOT_SPEED);
    } else if (node >= 10) {
      leftMotor.set(IntakeDesign2Constants.MID_SHOOT_SPEED);
      rightMotor.set(IntakeDesign2Constants.MID_SHOOT_SPEED);
    } else {
      leftMotor.set(IntakeDesign2Constants.LOW_SHOOT_SPEED);
      rightMotor.set(IntakeDesign2Constants.LOW_SHOOT_SPEED);
    }
  }
}
