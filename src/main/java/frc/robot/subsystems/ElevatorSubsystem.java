// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final WPI_TalonFX elevatorMotor;
  private final CANCoder elevatorEncoder; 

  private final DigitalInput bottomLimitSwitch;
  private final DigitalInput topLimitSwitch;

  private final PIDController elevatorPID = 
    new PIDController(
      ElevatorConstants.pElevator, 
      ElevatorConstants.iElevator, 
      ElevatorConstants.dElevator
    );
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    // Initialize motor
    elevatorMotor = new WPI_TalonFX(ElevatorConstants.elevatorMotorPort);
    elevatorMotor.setInverted(ElevatorConstants.elevatorInverted);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    // Initialize encoder
    elevatorEncoder = new CANCoder(ElevatorConstants.elevatorEncoderPort);
    elevatorEncoder.configSensorDirection(false);

    //Initialize limit switches
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchPort);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchPort);

  }



  public double getEncoderValue() {
    return elevatorEncoder.getPosition();
  }

  public boolean bottomLimitSwitchPressed() {
    return bottomLimitSwitch.get();
  }

  public boolean topLimitSwitchPressed() {
    return topLimitSwitch.get();
  }
 
  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public void stopElevator() {
    elevatorMotor.set(ControlMode.PercentOutput, 0.0);
  }

}
