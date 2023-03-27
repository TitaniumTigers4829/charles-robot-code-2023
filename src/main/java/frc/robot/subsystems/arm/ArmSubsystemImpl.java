package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.extras.SmartDashboardLogger;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final WPI_CANCoder rotationEncoder;
  private final WPI_TalonFX leaderRotationMotor;
  private final WPI_TalonFX followerRotationMotor;
  private final WPI_TalonFX extensionMotor;
  // private final DoubleSolenoid extensionLockSolenoid;

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
    rotationEncoder = new WPI_CANCoder(ArmConstants.ROTATION_ENCODER_ID, Constants.CANIVORE_CAN_BUS_STRING);
    leaderRotationMotor = new WPI_TalonFX(ArmConstants.LEADER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    followerRotationMotor = new WPI_TalonFX(ArmConstants.FOLLOWER_ROTATION_MOTOR_ID, Constants.CANIVORE_CAN_BUS_STRING);
    extensionMotor = new WPI_TalonFX(ArmConstants.EXTENSION_MOTOR_ID);
    // extensionLockSolenoid = new DoubleSolenoid(Constants.PNEUMATICS_MODULE_TYPE, ArmConstants.EXTENSION_LOCK_ENGAGED_ID, ArmConstants.EXTENSION_LOCK_DISENGAGED_ID);
    
    rotationEncoder.configMagnetOffset(ArmConstants.ROTATION_ENCODER_OFFSET);
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
    rotationEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);

    leaderRotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0, 0);
    leaderRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    leaderRotationMotor.config_kP(0, ArmConstants.ROTATION_P);
    leaderRotationMotor.config_kI(0, ArmConstants.ROTATION_I);
    leaderRotationMotor.config_kD(0, ArmConstants.ROTATION_D);
    leaderRotationMotor.configMotionCruiseVelocity(ArmConstants.ROTATION_MAX_VELOCITY_ENCODER_UNITS);
    leaderRotationMotor.configMotionAcceleration(ArmConstants.ROTATION_MAX_ACCELERATION_ENCODER_UNITS);
    leaderRotationMotor.configMotionSCurveStrength(ArmConstants.ROTATION_SMOOTHING);
    leaderRotationMotor.configAllowableClosedloopError(0, ArmConstants.ROTATION_TOLERANCE_ENCODER_UNITS);
    leaderRotationMotor.configForwardSoftLimitThreshold(ArmConstants.MAX_ROTATION_ENCODER_UNITS);
    leaderRotationMotor.configForwardSoftLimitEnable(true);
    leaderRotationMotor.configReverseSoftLimitThreshold(ArmConstants.MIN_ROTATION_ENCODER_UNITS);
    leaderRotationMotor.configReverseSoftLimitEnable(true);
    leaderRotationMotor.configNeutralDeadband(Constants.MIN_FALCON_DEADBAND);

    leaderRotationMotor.setInverted(ArmConstants.LEADER_ROTATION_MOTOR_INVERTED);
    leaderRotationMotor.setNeutralMode(NeutralMode.Brake);

    followerRotationMotor.setInverted(InvertType.OpposeMaster);
    followerRotationMotor.setNeutralMode(NeutralMode.Brake);
    followerRotationMotor.setSensorPhase(true);    
    followerRotationMotor.follow(leaderRotationMotor);

    extensionMotor.setInverted(ArmConstants.EXTENSION_MOTOR_INVERTED);
    extensionMotor.setNeutralMode(NeutralMode.Coast);
    extensionMotor.configNeutralDeadband(Constants.MIN_FALCON_DEADBAND);
    extensionMotor.configReverseSoftLimitThreshold(1 * ArmConstants.EXTENSION_METERS_TO_MOTOR_POS);
    extensionMotor.configReverseSoftLimitEnable(true);
  }

  @Override
  public void setRotation(double desiredAngle) {
    leaderRotationMotor.set(ControlMode.MotionMagic, desiredAngle * Constants.DEGREES_TO_CANCODER_UNITS);
  }

  @Override
  public double getRotation() {
    return rotationEncoder.getAbsolutePosition();
  }

  @Override
  public void resetExtensionEncoder() {
    extensionMotor.setSelectedSensorPosition(0);
  }

  @Override
  public double getExtension() {
    return -1 * extensionMotor.getSelectedSensorPosition()* ArmConstants.EXTENSION_MOTOR_POS_TO_METERS;
  }

  @Override
  public void setExtension(double extension) {
    double PIDOutput = extensionSpeedPIDController.calculate(getExtension(), extension);
    setExtensionSpeed(PIDOutput);
  }

  public void syncRotationEncoders() {
    leaderRotationMotor.setSelectedSensorPosition(getRotation() * ArmConstants.ARM_DEGREES_TO_FALCON_UNITS);
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
    return rotationEncoder.getVelocity();
  }

  @Override
  public void setRotationSpeed(double speed) {
    leaderRotationMotor.set(speed / 2);
  }

  @Override
  public double getExtensionSpeed() {
    // Convert motor rotation units (2048 for 1 full rotation) to number of rotations
    return -extensionMotor.getSelectedSensorVelocity() * ArmConstants.EXTENSION_MOTOR_POS_TO_METERS * 10.0;
    
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

  @Override
  public void setExtensionMotorNeutralMode(NeutralMode neutralMode) {
    extensionMotor.setNeutralMode(neutralMode);
  }

  @Override
  public void resetExtensionController() {
    extensionSpeedPIDController.reset(getExtension(), getExtensionSpeed());
  }

  @Override
  public void periodic() {
    SmartDashboardLogger.debugNumber("Arm rotation", getRotation());
    SmartDashboardLogger.debugNumber("Arm extension", getExtension());    
  }
}
