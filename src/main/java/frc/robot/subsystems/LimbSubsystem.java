// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimbConstants;
import frc.robot.Constants.LimbConstants.WristConstants;
import frc.robot.Constants.LimbConstants.ClawConstants;

public class ArmSubsystem extends SubsystemBase {

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private final DoubleSolenoid clawSolenoid;
  private final DoubleSolenoid armSolenoid;
  private final WPI_TalonFX wristMotor;
  private final CANCoder wristEncoder;

  private final ProfiledPIDController wristPID =
          new ProfiledPIDController(
                  Constants.ElevatorConstants.pElevator,
                  Constants.ElevatorConstants.iElevator,
                  Constants.ElevatorConstants.dElevator,
                  new TrapezoidProfile.Constraints(WristConstants.wristMaxVelocity, WristConstants.wristMaxAcceleration)
          );

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {


    //Initialize Motor
    wristMotor = new WPI_TalonFX(WristConstants.wristMotorID);
    wristMotor.setInverted(WristConstants.isMotorInverted);
    wristMotor.setNeutralMode(NeutralMode.Brake);

    // Initialize Encoder
    wristEncoder = new CANCoder(WristConstants.wristEncoderID);
    wristEncoder.configSensorDirection(false);

    // Initialize Solenoids
    armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            LimbConstants.ArmConstants.armForward,
            LimbConstants.ArmConstants.armBackward
    );
    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ClawConstants.solenoidForward,
            ClawConstants.solenoidBackward
    );
    CloseClaw();
  }

  public void CloseClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void OpenClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void ExtendArm() {
    armSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void SetWristMotor(double speed) {
    wristMotor.set(speed);
  }

  public void GetWristMotor(double speed) {
    wristMotor.get();
  }

  public void StopWristMotor() {
    wristMotor.stopMotor();
  }


}
