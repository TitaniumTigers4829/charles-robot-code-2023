package frc.robot.subsystems.arm;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Conversions;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.SmartDashboardLogger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final CANcoder rotationEncoder;
  private final TalonFX leaderRotationMotor;
  private final TalonFX followerRotationMotor;
  private final TalonFX extensionMotor;
  // private final DoubleSolenoid extensionLockSolenoid;

  private final StatusSignal<Double> rotation;
  private final StatusSignal<Double> rotationSpeed;
  private final StatusSignal<Double> extensionPos;
  private final StatusSignal<Double> extensionSpeed;

  private final ProfiledPIDController extensionSpeedPIDController = new ProfiledPIDController(
    ArmConstants.EXTENSION_P,
    ArmConstants.EXTENSION_I,
    ArmConstants.EXTENSION_D,
    ArmConstants.EXTENSION_CONSTRAINTS
  );

  /** 
   * Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    rotationEncoder = new CANcoder(ArmConstants.ROTATION_ENCODER_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    leaderRotationMotor = new TalonFX(ArmConstants.LEADER_ROTATION_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    followerRotationMotor = new TalonFX(ArmConstants.FOLLOWER_ROTATION_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    extensionMotor = new TalonFX(ArmConstants.EXTENSION_MOTOR_ID);

    rotation = rotationEncoder.getAbsolutePosition();
    rotationSpeed = rotationEncoder.getVelocity();
    extensionPos = extensionMotor.getPosition();
    extensionSpeed = extensionMotor.getVelocity();
    // extensionLockSolenoid = new DoubleSolenoid(Constants.PNEUMATICS_MODULE_TYPE, ArmConstants.EXTENSION_LOCK_ENGAGED_ID, ArmConstants.EXTENSION_LOCK_DISENGAGED_ID);
    rotationEncoder.getConfigurator().apply(new CANcoderConfiguration(), HardwareConstants.TIMEOUT_MS);
    CANcoderConfiguration rotationEncoderConfigs = new CANcoderConfiguration();
    rotationEncoderConfigs.MagnetSensor.MagnetOffset = ArmConstants.ROTATION_ENCODER_OFFSET;
    rotationEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
    rotationEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; //idk
    rotationEncoder.getPosition().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    rotationEncoder.getVelocity().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    
    rotationEncoder.getConfigurator().apply(rotationEncoderConfigs);
    // rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, HardwareConstants.TIMEOUT_MS);

    leaderRotationMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration talonfxConfigsArm = new TalonFXConfiguration();
    leaderRotationMotor.getConfigurator().refresh(talonfxConfigsArm, HardwareConstants.TIMEOUT_MS);
    leaderRotationMotor.getPosition().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    leaderRotationMotor.getVelocity().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    Slot0Configs slot0ConfigsArm = talonfxConfigsArm.Slot0;
    slot0ConfigsArm.kV = 0.0;
    slot0ConfigsArm.kS = 0.0;
    slot0ConfigsArm.kP = ArmConstants.ROTATION_P;
    slot0ConfigsArm.kI = ArmConstants.ROTATION_I;
    slot0ConfigsArm.kD = ArmConstants.ROTATION_D;

    CurrentLimitsConfigs currentLimitsConfigsArm = talonfxConfigsArm.CurrentLimits;
    currentLimitsConfigsArm.SupplyCurrentLimit = 60;
    currentLimitsConfigsArm.SupplyCurrentLimitEnable = true;
    currentLimitsConfigsArm.SupplyCurrentThreshold = 65;
    currentLimitsConfigsArm.SupplyTimeThreshold = 0.1;
    currentLimitsConfigsArm.StatorCurrentLimit = 60;
    currentLimitsConfigsArm.StatorCurrentLimitEnable = true;

    talonfxConfigsArm.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonfxConfigsArm.MotorOutput.Inverted = ArmConstants.LEADER_ROTATION_MOTOR_INVERTED;
    
    MotionMagicConfigs motionMagicConfigs = talonfxConfigsArm.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ROTATION_MAX_VELOCITY_ENCODER_UNITS; 
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.ROTATION_MAX_ACCELERATION_ENCODER_UNITS; 
    // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigsArm = talonfxConfigsArm.SoftwareLimitSwitch;

    softwareLimitSwitchConfigsArm.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigsArm.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigsArm.ForwardSoftLimitThreshold = ArmConstants.MAX_ROTATION;
    softwareLimitSwitchConfigsArm.ReverseSoftLimitThreshold = ArmConstants.MIN_ROTATION;
    
    talonfxConfigsArm.Feedback.FeedbackRemoteSensorID = rotationEncoder.getDeviceID();
    talonfxConfigsArm.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //apply configs
    leaderRotationMotor.getConfigurator().apply(talonfxConfigsArm, HardwareConstants.TIMEOUT_MS);
    
    // leaderRotationMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);
    followerRotationMotor.getConfigurator().apply(new TalonFXConfiguration(), HardwareConstants.TIMEOUT_MS);
    TalonFXConfiguration talonfxConfigsFollower = new TalonFXConfiguration();
    talonfxConfigsFollower.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // followerRotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    followerRotationMotor.setControl(new Follower(leaderRotationMotor.getDeviceID(), true));
    followerRotationMotor.getConfigurator().apply(talonfxConfigsFollower, HardwareConstants.TIMEOUT_MS);


    extensionMotor.getConfigurator().apply(new TalonFXConfiguration(), HardwareConstants.TIMEOUT_MS);
    TalonFXConfiguration talonfxConfigsExtend = new TalonFXConfiguration();
    extensionMotor.getConfigurator().refresh(talonfxConfigsExtend);
    extensionMotor.getPosition().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    extensionMotor.getVelocity().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    talonfxConfigsExtend.Slot0.kV = 0.0;
    talonfxConfigsExtend.Slot0.kS = 0.0;
    talonfxConfigsExtend.Slot0.kP = ArmConstants.EXTENSION_P;
    talonfxConfigsExtend.Slot0.kI = ArmConstants.EXTENSION_I;
    talonfxConfigsExtend.Slot0.kD = ArmConstants.EXTENSION_D;

    talonfxConfigsExtend.CurrentLimits.SupplyCurrentLimit = 60;
    talonfxConfigsExtend.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonfxConfigsExtend.CurrentLimits.SupplyCurrentThreshold = 65;
    talonfxConfigsExtend.CurrentLimits.SupplyTimeThreshold = 0.1;
    talonfxConfigsExtend.CurrentLimits.StatorCurrentLimit = 60;
    talonfxConfigsExtend.CurrentLimits.StatorCurrentLimitEnable = true;

    talonfxConfigsExtend.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonfxConfigsExtend.MotorOutput.Inverted = ArmConstants.EXTENSION_MOTOR_INVERTED;

    talonfxConfigsExtend.Feedback.RotorToSensorRatio =  ArmConstants.EXTENSION_MOTOR_GEAR_RATIO;
    
    MotionMagicConfigs extensionMotionMagicConfigs = talonfxConfigsExtend.MotionMagic;
    extensionMotionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ROTATION_MAX_VELOCITY_ENCODER_UNITS;
    extensionMotionMagicConfigs.MotionMagicAcceleration = ArmConstants.ROTATION_MAX_ACCELERATION_ENCODER_UNITS;
    // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    SoftwareLimitSwitchConfigs extensionSoftwareLimitSwitchConfigs = talonfxConfigsExtend.SoftwareLimitSwitch;

    extensionSoftwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    extensionSoftwareLimitSwitchConfigs.ReverseSoftLimitThreshold = ArmConstants.MAX_EXTENSION_METERS * ArmConstants.EXTENSION_METERS_TO_MOTOR_POS;
    //apply configs
    extensionMotor.getConfigurator().apply(talonfxConfigsExtend, HardwareConstants.TIMEOUT_MS);
    // extensionMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);


  }

  @Override
  public void setRotation(double desiredAngle) {
    // leaderRotationMotor.set(ControlMode.MotionMagic, desiredAngle * Conversions.DEGREES_TO_CANCODER_UNITS);
    MotionMagicVoltage mmDutyCycle = new MotionMagicVoltage(desiredAngle); // this works maybe
    leaderRotationMotor.setControl(mmDutyCycle);
    // followerRotationMotor.follow(leaderRotationMotor);
  }

  @Override
  public double getRotation() {
    rotation.refresh();
    return rotation.getValue();
  }

  @Override
  public void resetExtensionEncoder() {
    extensionMotor.setRotorPosition(0);
  }

  @Override
  public double getExtension() {
    extensionPos.refresh();
    return -1 * extensionPos.getValue()* ArmConstants.EXTENSION_MOTOR_POS_TO_METERS;
  }

  @Override
  public void setExtension(double extension) {
    double PIDOutput = extensionSpeedPIDController.calculate(getExtension(), extension);
    setExtensionSpeed(PIDOutput);
  }

  @Override
  public void lockExtensionSolenoid() {
    // extensionLockSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void unlockExtensionSolenoid() {
    // extensionLockSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public double getRotationSpeed() {
    rotationSpeed.refresh();
    return rotationSpeed.getValue();
  }

  @Override
  public void setRotationSpeed(double speed) {
    leaderRotationMotor.set(speed / 2);
  }

  @Override
  public double getExtensionSpeed() {
    // Convert motor rotation units (2048 for 1 full rotation) to number of rotations
    extensionSpeed.refresh();
    return -extensionSpeed.getValue() * ArmConstants.EXTENSION_MOTOR_POS_TO_METERS * 10.0;
    
  }

  @Override
  public void setExtensionSpeed(double speed) {
    extensionMotor.set(speed);
  }

  @Override
  public double getTorqueFromGravity() {
    // Torque = mg(COM Distance*sin(theta) - r*sin(theta))
    // double centerOfMassDistance = (0.4659 * getExtension()) + 0.02528; // This is the equation fit to COM distance
    double centerOfMassDistance = (0.4659 * getExtension()); // This is the equation fit to COM distance
    double theta = Math.toRadians(getRotation() - 90); // The angle of the arm is 0 when it's pointing down
    return ArmConstants.ARM_WEIGHT_NEWTONS * 
      (centerOfMassDistance * Math.cos(theta) - ArmConstants.ARM_AXIS_OF_ROTATION_RADIUS * Math.sin(theta));
  }

  // @Override
  // public void setExtensionMotorNeutralMode(NeutralMode neutralMode) {
  //   extensionMotor.setNeutralMode(neutralMode);
  // }

  @Override
  public void resetExtensionController() {
    extensionSpeedPIDController.reset(getExtension(), getExtensionSpeed());
  }

  @Override
  public void periodic() {
    SmartDashboardLogger.debugNumber("Arm Rotation", getRotation());
    SmartDashboardLogger.debugNumber("Arm Extension", getExtension());
  }

  @Override
  public void setExtensionMotorNeutralMode(NeutralMode neutralMode) {
    // TODO Auto-generated method stub
    
  }
}
