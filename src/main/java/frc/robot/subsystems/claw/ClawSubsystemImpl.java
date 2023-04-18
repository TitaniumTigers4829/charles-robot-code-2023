package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.SmartDashboardLogger;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final WPI_TalonFX wristMotor;
  private final WPI_TalonFX intakeMotor;
  private final DoubleSolenoid clawSolenoid;

  private boolean isClawClosed;
  private boolean isManualControl = false;

  public ClawSubsystemImpl() {
    wristMotor = new WPI_TalonFX(ClawConstants.WRIST_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);
    intakeMotor = new WPI_TalonFX(ClawConstants.INTAKE_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);

    wristMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    
    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, HardwareConstants.TIMEOUT_MS);

    wristMotor.config_kP(0, ClawConstants.WRIST_P, HardwareConstants.TIMEOUT_MS);
    wristMotor.config_kI(0, ClawConstants.WRIST_I, HardwareConstants.TIMEOUT_MS);
    wristMotor.config_kD(0, ClawConstants.WRIST_D, HardwareConstants.TIMEOUT_MS);
    wristMotor.config_kF(0, ClawConstants.WRIST_F, HardwareConstants.TIMEOUT_MS);
    wristMotor.configAllowableClosedloopError(0, ClawConstants.WRIST_TOLERANCE, HardwareConstants.TIMEOUT_MS);

    wristMotor.configMotionCruiseVelocity(ClawConstants.WRIST_MAX_VELOCITY_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    wristMotor.configMotionAcceleration(ClawConstants.WRIST_MAX_ACCELERATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    wristMotor.configMotionSCurveStrength(ClawConstants.WRIST_SMOOTHING, HardwareConstants.TIMEOUT_MS);


    wristMotor.configForwardSoftLimitThreshold(ClawConstants.MAX_WRIST_ROTATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    wristMotor.configForwardSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);
    wristMotor.configReverseSoftLimitThreshold(ClawConstants.MIN_WRIST_ROTATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    wristMotor.configReverseSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);

    wristMotor.setInverted(ClawConstants.WRIST_MOTOR_INVERTED);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setSelectedSensorPosition(0);
    wristMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND);

    wristMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    wristMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    intakeMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);

    intakeMotor.setInverted(ClawConstants.INTAKE_MOTOR_INVERTED);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);

    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    clawSolenoid = new DoubleSolenoid(
      HardwareConstants.PNEUMATICS_MODULE_TYPE,
      ClawConstants.CLAW_FORWARD,
      ClawConstants.CLAW_BACKWARD
    );

    isManualControl = false;
  }

  @Override
  public void close() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    isClawClosed = true;
  }

  @Override
  public void open() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);    
    isClawClosed = false;
  }

  @Override
  public boolean isClawClosed() { 
    return isClawClosed;
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override 
  public void setWristMotorSpeed(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public double getWristAngle() {
    return wristMotor.getSelectedSensorPosition() * ClawConstants.WRIST_POS_TO_DEG;
  }

  @Override
  public void zeroWristEncoder() {
    wristMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void setWristPosition(double angle) {
    wristMotor.set(ControlMode.MotionMagic, angle * ClawConstants.DEG_TO_WRIST_POS);
  }


  @Override
  public boolean isManualControl() {
    return isManualControl;
  }

  @Override
  public void toggleControlMode() {
    isManualControl = !isManualControl;
  }
  
  @Override
  public void periodic() {
    SmartDashboardLogger.infoBoolean("isManual", isManualControl);
  }
}
