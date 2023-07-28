package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.SmartDashboardLogger;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final TalonFX wristMotor;
  private final TalonFX intakeMotor;
  private final DoubleSolenoid clawSolenoid;

  private boolean isClawClosed;
  private boolean isManualControl = false;

  private StatusSignal<Double> wristPosition;

  

  public ClawSubsystemImpl() {
    wristMotor = new TalonFX(ClawConstants.WRIST_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);
    intakeMotor = new TalonFX(ClawConstants.INTAKE_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);

    wristPosition = wristMotor.getPosition();
    wristMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration talonfxConfigsWrist= new TalonFXConfiguration();
    wristMotor.getConfigurator().refresh(talonfxConfigsWrist);
    wristMotor.getPosition().setUpdateFrequency(5);
    wristMotor.getVelocity().setUpdateFrequency(5);
    talonfxConfigsWrist.Slot0.kV = 0.0;
    talonfxConfigsWrist.Slot0.kS = 0.0;
    talonfxConfigsWrist.Slot0.kP = ClawConstants.WRIST_P;
    talonfxConfigsWrist.Slot0.kI = ClawConstants.WRIST_I;
    talonfxConfigsWrist.Slot0.kD = ClawConstants.WRIST_D;

    talonfxConfigsWrist.CurrentLimits.SupplyCurrentLimit = 60;
    talonfxConfigsWrist.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonfxConfigsWrist.CurrentLimits.SupplyCurrentThreshold = 65;
    talonfxConfigsWrist.CurrentLimits.SupplyTimeThreshold = 0.1;
    talonfxConfigsWrist.CurrentLimits.StatorCurrentLimit = 60;
    talonfxConfigsWrist.CurrentLimits.StatorCurrentLimitEnable = true;

    talonfxConfigsWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonfxConfigsWrist.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonfxConfigsWrist.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    
    MotionMagicConfigs wristMotionMagicConfigs = talonfxConfigsWrist.MotionMagic;
    wristMotionMagicConfigs.MotionMagicCruiseVelocity = ClawConstants.WRIST_MAX_VELOCITY_ENCODER_UNITS; // Target cruise velocity of 80 rps
    wristMotionMagicConfigs.MotionMagicAcceleration = ClawConstants.WRIST_MAX_ACCELERATION_ENCODER_UNITS; // Target acceleration of 160 rps/s (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    SoftwareLimitSwitchConfigs wristSoftwareLimitSwitchConfigs = talonfxConfigsWrist.SoftwareLimitSwitch;

    talonfxConfigsWrist.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonfxConfigsWrist.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonfxConfigsWrist.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClawConstants.MAX_WRIST_ROTATION_ENCODER_UNITS;
    talonfxConfigsWrist.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClawConstants.MIN_WRIST_ROTATION_ENCODER_UNITS;

    wristMotor.setRotorPosition(0, HardwareConstants.TIMEOUT_MS);//wristMotor.setSelectedSensorPosition(0);
    //apply configs
    wristMotor.getConfigurator().apply(talonfxConfigsWrist, HardwareConstants.TIMEOUT_MS);
    

    // wristMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND);

    // wristMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // wristMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    intakeMotor.getConfigurator().apply(new TalonFXConfiguration(), HardwareConstants.TIMEOUT_MS);
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    intakeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(intakeConfigs, HardwareConstants.TIMEOUT_MS);
    // intakeMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);

    // intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

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
    wristPosition.refresh();
    return wristPosition.getValue(); 
  }

  @Override
  public void zeroWristEncoder() {
    wristMotor.setRotorPosition(0, HardwareConstants.TIMEOUT_MS);
  }

  @Override
  public void setWristPosition(double angle) {
    MotionMagicDutyCycle mmDutyCycle = new MotionMagicDutyCycle(angle);
    wristMotor.setControl(mmDutyCycle);
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
