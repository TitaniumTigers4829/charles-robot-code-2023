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
import com.ctre.phoenix.sensors.CANCoder;
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

  private final CANCoder turnEncoder;
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;

  private final ProfiledPIDController turnPIDController =
    new ProfiledPIDController(
      ModuleConstants.TURN_P,
      ModuleConstants.TURN_I,
      ModuleConstants.TURN_D,
      ModuleConstants.TURN_CONSTRAINTS
    );

  private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
    DriveConstants.TURN_S, 
    DriveConstants.TURN_V, 
    DriveConstants.TURN_A
  );

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
    ModuleConstants.DRIVE_S,
    ModuleConstants.DRIVE_V,
    ModuleConstants.DRIVE_A
  );

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
    
    turnEncoder = new CANCoder(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    driveMotor = new WPI_TalonFX(driveMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnMotor = new WPI_TalonFX(turnMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
        
    turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, HardwareConstants.TIMEOUT_MS);
    turnEncoder.configMagnetOffset(-angleZero, HardwareConstants.TIMEOUT_MS);
    turnEncoder.configSensorDirection(encoderReversed, HardwareConstants.TIMEOUT_MS);

    driveMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, HardwareConstants.TIMEOUT_MS);
    driveMotor.config_kF(0, ModuleConstants.DRIVE_F, HardwareConstants.TIMEOUT_MS);
    driveMotor.config_kP(0, ModuleConstants.DRIVE_P, HardwareConstants.TIMEOUT_MS);
    driveMotor.config_kI(0, ModuleConstants.DRIVE_I, HardwareConstants.TIMEOUT_MS);
    driveMotor.config_kD(0, ModuleConstants.DRIVE_D, HardwareConstants.TIMEOUT_MS);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setInverted(driveReversed);
    driveMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND * 10, HardwareConstants.TIMEOUT_MS);
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1), HardwareConstants.TIMEOUT_MS);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1), HardwareConstants.TIMEOUT_MS);
    driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    turnMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.setInverted(true);
    turnMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);
    turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1), HardwareConstants.TIMEOUT_MS);
    turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1), HardwareConstants.TIMEOUT_MS);
    turnMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    turnMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
  public double getModuleHeading() {
    return this.turnEncoder.getAbsolutePosition() % 360;
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double speedMetersPerSecond = ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotor.getSelectedSensorVelocity();
    double turnRadians = (Math.PI / 180) * turnEncoder.getAbsolutePosition();
    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(turnRadians));
  }

  public SwerveModulePosition getPosition() {
    double position = ModuleConstants.DRIVE_TO_METERS * driveMotor.getSelectedSensorPosition();
    Rotation2d rotation = Rotation2d.fromDegrees(getCANCoderABS());
    return new SwerveModulePosition(position, rotation);
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRadians = getTurnRadians();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));
    SmartDashboard.putNumber(name + " difference in speed", optimizedDesiredState.speedMetersPerSecond - getState().speedMetersPerSecond);

    // Converts meters per second to rpm
    double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60 
      * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
      
    // Converts rpm to encoder units per 100 milliseconds
    double desiredDriveEncoderUnitsPer100MS = desiredDriveRPM / 600.0 * 2048;

    // Sets the drive motor's speed using the built in pid controller
    driveMotor.set(ControlMode.Velocity, desiredDriveEncoderUnitsPer100MS, 
      DemandType.ArbitraryFeedForward, driveFeedForward.calculate(optimizedDesiredState.speedMetersPerSecond));

    // Calculate the turning motor output from the turn PID controller.
    double turnOutput =
      turnPIDController.calculate(turnRadians, optimizedDesiredState.angle.getRadians())
        + turnFeedForward.calculate(turnPIDController.getSetpoint().velocity);
        turnMotor.set(turnOutput / 12);
  }

  public double getTurnRadians() {
    return ((2 * Math.PI) / 360) * turnEncoder.getAbsolutePosition();
  }

  public double getAbsolutePosition() {
    return turnEncoder.getAbsolutePosition();
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position
   */
  public double getCANCoderABS(){
    return turnEncoder.getAbsolutePosition();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    turnEncoder.setPosition(0);
    driveMotor.setSelectedSensorPosition(0);
  }

  public void periodicFunction() {
  }
}