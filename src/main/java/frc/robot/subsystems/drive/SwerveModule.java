package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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
    InvertedValue encoderReversed,
    InvertedValue driveReversed,
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

    TalonFXConfiguration talonfxConfigs = new TalonFXConfiguration();

    talonfxConfigs.Slot0.kV = 0.0;
    talonfxConfigs.Slot0.kS = 0.0;
    talonfxConfigs.Slot0.kP = ModuleConstants.DRIVE_P;
    talonfxConfigs.Slot0.kI = ModuleConstants.DRIVE_I;
    talonfxConfigs.Slot0.kD = ModuleConstants.DRIVE_D;

    talonfxConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = 65;
    talonfxConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
    talonfxConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonfxConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveMotor.getConfigurator().refresh(talonfxConfigs);
    driveMotor.getPosition().setUpdateFrequency(5, HardwareConstants.TIMEOUT_MS);
    driveMotor.getVelocity().setUpdateFrequency(5);
    //apply configs
    driveMotor.getConfigurator().apply(talonfxConfigs, HardwareConstants.TIMEOUT_MS);
    // driveMotor.setRotorPosition(0.0);

    //factory defaults turn motor
    turnMotor.getConfigurator().apply(new TalonFXConfiguration(), HardwareConstants.TIMEOUT_MS);

    TalonFXConfiguration talonfxConfigsTurn = new TalonFXConfiguration();

    talonfxConfigsTurn.Slot0.kV = 0.0;
    talonfxConfigsTurn.Slot0.kS = 0.0;
    talonfxConfigsTurn.Slot0.kP = ModuleConstants.TURN_P;
    talonfxConfigsTurn.Slot0.kI = ModuleConstants.TURN_I;
    talonfxConfigsTurn.Slot0.kD = ModuleConstants.TURN_D;

    talonfxConfigsTurn.CurrentLimits.SupplyCurrentLimit = 60;
    talonfxConfigsTurn.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonfxConfigsTurn.CurrentLimits.SupplyCurrentThreshold = 65;
    talonfxConfigsTurn.CurrentLimits.SupplyTimeThreshold = 0.1;
    talonfxConfigsTurn.CurrentLimits.StatorCurrentLimit = 60;
    talonfxConfigsTurn.CurrentLimits.StatorCurrentLimitEnable = true;

    // talonfxConfigs.MotorOutput.Inverted = driveReversed;
    talonfxConfigsTurn.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //enables continuous input
    talonfxConfigsTurn.ClosedLoopGeneral.ContinuousWrap = true;
    
    talonfxConfigsTurn.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

    turnMotor.getPosition().setUpdateFrequency(5);
    turnMotor.getVelocity().setUpdateFrequency(5);
    // set Motion Magic settings
    MotionMagicConfigs turnMotionMagicConfigs = talonfxConfigsTurn.MotionMagic;
    turnMotionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    turnMotionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    turnMotionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //applies configs
    turnMotor.getConfigurator().apply(talonfxConfigsTurn);
    // turnMotor.setRotorPosition(0.0);


    //idk how to do this one:
    // turnMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);

    
  }

  /**
   * @param currentAngle what the controller currently reads (radians)
   * @param targetAngleSetpoint the desired angle (radians)
   * @return the target angle in controller's scope (radians)
   */
  public static double calculateContinuousInputSetpoint(double currentAngle, double targetAngleSetpoint) {
    targetAngleSetpoint = Math.IEEEremainder(targetAngleSetpoint, Math.PI * 2);

    double remainder = currentAngle % (Math.PI * 2);
    double adjustedAngleSetpoint = targetAngleSetpoint + (currentAngle - remainder);

    // We don't want to rotate over 180 degrees, so just rotate the other way (add a
    // full rotation)
    if (adjustedAngleSetpoint - currentAngle > Math.PI) {
        adjustedAngleSetpoint -= Math.PI * 2;
    } else if (adjustedAngleSetpoint - currentAngle < -Math.PI) {
        adjustedAngleSetpoint += Math.PI * 2;
    }

    return adjustedAngleSetpoint;
  }

  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  // public double getModuleHeading() {
  //   return turnEncoder.getPosition() % 360;
  // }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  // public SwerveModuleState getState() {
  //   double speedMetersPerSecond = ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotor.getRotorVelocity();
  //   double turnRadians = (Math.PI / 180) * turnEncoder.getAbsolutePosition();
  //   return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(turnRadians));
  // }
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
    double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

    double distance = driveRotations / ModuleConstants.ROTATIONS_PER_METER;
    internalState.distanceMeters = distance;
    Rotation2d angle = Rotation2d.fromRotations(angleRotations);
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
    turnMotor.setControl(angleSetter.withPosition(angleToSetDeg).withFeedForward(DriveConstants.TURN_S));
    double velocityToSet = optimized.speedMetersPerSecond * ModuleConstants.ROTATIONS_PER_METER;
    driveMotor.setControl(driveSetter.withVelocity(velocityToSet).withFeedForward(ModuleConstants.DRIVE_S));
//// create a position closed-loop request, voltage output, slot 0 configs
// var request = new PositionVoltage(0).withSlot(0);

// // set position to 10 rotations
// m_talonFX.setControl(request.withPosition(10));
// create a velocity closed-loop request, voltage output, slot 0 configs
// var request = new VelocityVoltage(0).withSlot(0);

// // set velocity to 8 rps, add 0.5 V to overcome gravity
// m_talonFX.setControl(request.withVelocity(8).withFeedForward(0.5));
// in init function
// var talonFXConfigs = new TalonFXConfiguration();

// set slot 0 gains
// var slot0Configs = talonFXConfigs.Slot0Configs;
// slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
// slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
// slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
// slot0Configs.kI = 0; // no output for integrated error
// slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

// // set Motion Magic settings
// var motionMagicConfigs = talonFXConfigs.MotionMagicConfigs;
// motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
// motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
// motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

// m_talonFX.getConfigurator().apply(talonFXConfigs);
// create a Motion Magic request, voltage output, slot 0 configs
// var request = new MotionMagicVoltage(0).withSlot(0);

// // set position to 10 rotations
// m_talonFX.setControl(request.withPosition(10));
   
  }

  // public double getTurnRadians() {
  //   return ((2 * Math.PI) / 360) * turnEncoder.get
  //   // .getAbsolutePosition();
  // }

  // public double getAbsolutePosition() {
  //   return turnEncoder.getAbsolutePosition();
  // }

  // /**
  //  * Gets the current position of the CANCoder in relation to the magnet
  //  * @return current CANCoder position
  //  */
  // public double getCANCoderABS(){
  //   return turnEncoder.getAbsolutePosition();
  // }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    turnEncoder.setPosition(0,50);
    // driveMotor.setSelectedSensorPosition(0);
    driveMotor.setRotorPosition(0.0, 50);
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