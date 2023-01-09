// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimbConstants;
import frc.robot.Constants.LimbConstants.WristConstants;
import frc.robot.Constants.LimbConstants.ClawConstants;

public class LimbSubsystem extends SubsystemBase {

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
                  WristConstants.wristP,
                  WristConstants.wristI,
                  WristConstants.wristD,
                  new TrapezoidProfile.Constraints(WristConstants.wristMaxVelocity, WristConstants.wristMaxAcceleration)
          );

  private final SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(
            WristConstants.wristS,
            WristConstants.wristV,
            WristConstants.wristA
  );

  private boolean clawOpen = false;

  /** Creates a new ArmSubsystem. */
  public LimbSubsystem() {


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
    CloseArm();
  }

  private void CloseClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  private void OpenClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void ToggleClaw() {
    clawOpen = !clawOpen;
    if (clawOpen) {
      OpenClaw();
    } else {
      CloseClaw();
    }
  }

  public void ExtendArm() {
    armSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void CloseArm() {
    armSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void SetWristMotor(double speed) {
    wristMotor.set(speed);
  }

  public double GetEncoderValue() {
    return wristEncoder.getPosition();
  }

  public void StopWristMotor() {
    wristMotor.stopMotor();
  }

  /** Sets the desired rotation for the wrist.
   * @param rotation The desired rotation (in radians.)
   */
  public void SetDesiredWristRotation(double rotation) {

  }


}
