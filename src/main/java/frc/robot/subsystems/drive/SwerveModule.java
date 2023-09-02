package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANcoder turnEncoder;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  
  private StatusSignal<Double> drivePosition;
  private StatusSignal<Double> driveVelocity;
  private StatusSignal<Double> anglePosition;
  private StatusSignal<Double> angleVelocity;
  private BaseStatusSignal[] signals;
  

  private PositionVoltage angleSetter = new PositionVoltage(0).withSlot(0).withUpdateFreqHz(5); //with voltage comp
  private VelocityVoltage driveSetter = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(5); //with voltage comp

  private SwerveModulePosition internalState = new SwerveModulePosition();

  private String name;

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turnMotorChannel ID of the turn motor
   * @param turnEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param driveReversed is the drive motor reversed
   */
  public SwerveModule(
    int driveMotorChannel,
    int turnMotorChannel,
    int turnEncoderChannel,
    double angleZero,
    boolean encoderReversed,
    boolean driveReversed,
    String name
    ) {
    this.name = name;
    // set invert to CW+ and apply config change
    turnEncoder = new CANcoder(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    driveMotor = new TalonFX(driveMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnMotor = new TalonFX(turnMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);

    turnEncoder.getConfigurator().apply(new CANcoderConfiguration());
    CANcoderConfiguration angleEncoderConfigs = new CANcoderConfiguration();
    angleEncoderConfigs.MagnetSensor.MagnetOffset = -angleZero;
    angleEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
    angleEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; //idk
    turnEncoder.getConfigurator().apply(angleEncoderConfigs);


    //factory default drive motor
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration talonfxConfigsDrive = new TalonFXConfiguration();

    talonfxConfigsDrive.Slot0.kV = ModuleConstants.DRIVE_V;
    talonfxConfigsDrive.Slot0.kS = ModuleConstants.DRIVE_S;
    talonfxConfigsDrive.Slot0.kP = ModuleConstants.DRIVE_P;
    talonfxConfigsDrive.Slot0.kI = ModuleConstants.DRIVE_I;
    talonfxConfigsDrive.Slot0.kD = ModuleConstants.DRIVE_D;

    talonfxConfigsDrive.CurrentLimits.SupplyCurrentLimit = 60;
    talonfxConfigsDrive.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonfxConfigsDrive.CurrentLimits.SupplyCurrentThreshold = 65;
    talonfxConfigsDrive.CurrentLimits.SupplyTimeThreshold = 0.1;
    talonfxConfigsDrive.CurrentLimits.StatorCurrentLimit = 60;
    talonfxConfigsDrive.CurrentLimits.StatorCurrentLimitEnable = true;
    
    talonfxConfigsDrive.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonfxConfigsDrive.Feedback.RotorToSensorRatio = ModuleConstants.DRIVE_GEAR_RATIO;

    driveMotor.getConfigurator().refresh(talonfxConfigsDrive);
    driveMotor.getPosition().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    driveMotor.getVelocity().setUpdateFrequency(5,  HardwareConstants.TIMEOUT_MS);

    talonfxConfigsDrive.MotorOutput.Inverted = driveReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    //apply configs
    driveMotor.getConfigurator().apply(talonfxConfigsDrive, HardwareConstants.TIMEOUT_MS);
    // driveMotor.setRotorPosition(0.0);

    //factory defaults turn motor
    turnMotor.getConfigurator().apply(new TalonFXConfiguration(), HardwareConstants.TIMEOUT_MS);

    TalonFXConfiguration talonfxConfigsTurn = new TalonFXConfiguration();

    talonfxConfigsTurn.Slot0.kV = ModuleConstants.TURN_V;
    talonfxConfigsTurn.Slot0.kS = ModuleConstants.TURN_S;
    talonfxConfigsTurn.Slot0.kP = ModuleConstants.TURN_P;
    talonfxConfigsTurn.Slot0.kI = ModuleConstants.TURN_I;
    talonfxConfigsTurn.Slot0.kD = ModuleConstants.TURN_D;

    talonfxConfigsTurn.CurrentLimits.SupplyCurrentLimit = 60;
    talonfxConfigsTurn.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonfxConfigsTurn.CurrentLimits.SupplyCurrentThreshold = 65;
    talonfxConfigsTurn.CurrentLimits.SupplyTimeThreshold = 0.1;
    talonfxConfigsTurn.CurrentLimits.StatorCurrentLimit = 60;
    talonfxConfigsTurn.CurrentLimits.StatorCurrentLimitEnable = true;

    talonfxConfigsTurn.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //enables continuous input
    talonfxConfigsTurn.ClosedLoopGeneral.ContinuousWrap = true;
    
    talonfxConfigsTurn.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

    turnMotor.getConfigurator().refresh(talonfxConfigsTurn);
    turnMotor.getPosition().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    turnMotor.getVelocity().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    // set Motion Magic settings
    MotionMagicConfigs turnMotionMagicConfigs = talonfxConfigsTurn.MotionMagic;
    turnMotionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    turnMotionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    turnMotionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    talonfxConfigsTurn.Feedback.RotorToSensorRatio = ModuleConstants.TURN_GEAR_RATIO;
    //applies configs
    turnMotor.getConfigurator().apply(talonfxConfigsTurn, HardwareConstants.TIMEOUT_MS);
    // turnMotor.setRotorPosition(0.0);


    // turnMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    anglePosition = turnMotor.getPosition();
    angleVelocity = turnMotor.getVelocity();

    signals = new BaseStatusSignal[4];
    signals[0] = drivePosition;
    signals[1] = driveVelocity;
    signals[2] = anglePosition;
    signals[3] = angleVelocity;
    
  }

  /**
   * @param currentAngle what the controller currently reads (radians)
   * @param targetAngleSetpoint the desired angle (radians)
   * @return the target angle in controller's scope (radians)
   */
  // public static double calculateContinuousInputSetpoint(double currentAngle, double targetAngleSetpoint) {
  //   targetAngleSetpoint = Math.IEEEremainder(targetAngleSetpoint, Math.PI * 2);

  //   double remainder = currentAngle % (Math.PI * 2);
  //   double adjustedAngleSetpoint = targetAngleSetpoint + (currentAngle - remainder);

  //   // We don't want to rotate over 180 degrees, so just rotate the other way (add a
  //   // full rotation)
  //   if (adjustedAngleSetpoint - currentAngle > Math.PI) {
  //       adjustedAngleSetpoint -= Math.PI * 2;
  //   } else if (adjustedAngleSetpoint - currentAngle < -Math.PI) {
  //       adjustedAngleSetpoint += Math.PI * 2;
  //   }

  //   return adjustedAngleSetpoint;
  // }

  // /**
  //  * Gets the heading of the module
  //  * @return the absolute position of the CANCoder
  //  */
  // public double getModuleHeading() {
  //   return turnEncoder.getPosition() % 360;
  // }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  
  public SwerveModuleState getState(boolean refresh) {
    if(refresh) {
      drivePosition.refresh();
      driveVelocity.refresh();
      anglePosition.refresh();
      angleVelocity.refresh();
    }

    return new SwerveModuleState(driveVelocity.getValue(), Rotation2d.fromRotations(anglePosition.getValue()));
}


  // public SwerveModulePosition getPosition() {
  //   double position = ModuleConstants.DRIVE_TO_METERS * driveMotor.getSelectedSensorPosition();
  //   Rotation2d rotation = Rotation2d.fromDegrees(getCANCoderABS());
  //   return new SwerveModulePosition(position, rotation);
  // }
  public SwerveModulePosition getPosition(boolean refresh) {
    if(refresh) {
        drivePosition.refresh();
        driveVelocity.refresh();
        anglePosition.refresh();
        angleVelocity.refresh();
    }
    
    //gets the "latency compensated value" for the rotation
    double driveSignals = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angleSignals = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

    double distance = driveSignals / ModuleConstants.ROTATIONS_PER_METER;
    internalState.distanceMeters = distance;
    Rotation2d angle = Rotation2d.fromRotations(angleSignals);
    internalState.angle = angle;
    
    return internalState;
}


  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, internalState.angle);

    double angleToSetDeg = optimized.angle.getRotations();
    turnMotor.setControl(angleSetter.withPosition(angleToSetDeg));
    double velocityToSet = optimized.speedMetersPerSecond * ModuleConstants.ROTATIONS_PER_METER;
    driveMotor.setControl(driveSetter.withVelocity(velocityToSet));
   
  }

  // public double getTurnRadians() {
  //   return ((2 * Math.PI) / 360) * turnEncoder.getAbsolutePosition();
  // }

  // /**
  //  * Gets the current position of the CANCoder in relation to the magnet
  //  * @return current CANCoder position
  //  */
  public StatusSignal<Double> getCANCoderABS(){
    return turnEncoder.getAbsolutePosition();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    turnEncoder.setPosition(0,HardwareConstants.TIMEOUT_MS);
    driveMotor.setRotorPosition(0.0, HardwareConstants.TIMEOUT_MS);
  }
  public void stop() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
}

  public void periodicFunction() {}

  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}