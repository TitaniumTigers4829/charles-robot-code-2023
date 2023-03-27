package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
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
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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
    
    turnEncoder = new CANCoder(turnEncoderChannel, Constants.CANIVORE_CAN_BUS_STRING);
    driveMotor = new WPI_TalonFX(driveMotorChannel, Constants.CANIVORE_CAN_BUS_STRING);
    turnMotor = new WPI_TalonFX(turnMotorChannel, Constants.CANIVORE_CAN_BUS_STRING);
        
    turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    turnEncoder.configMagnetOffset(-angleZero);
    turnEncoder.configSensorDirection(encoderReversed);

    driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    driveMotor.config_kF(0, ModuleConstants.DRIVE_F);
    driveMotor.config_kP(0, ModuleConstants.DRIVE_P);
    driveMotor.config_kI(0, ModuleConstants.DRIVE_I);
    driveMotor.config_kD(0, ModuleConstants.DRIVE_D);     
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setInverted(driveReversed);
    driveMotor.configNeutralDeadband(Constants.MIN_FALCON_DEADBAND);
    driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));

    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.setInverted(true);
    turnMotor.configNeutralDeadband(Constants.MIN_FALCON_DEADBAND);
    turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
    turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));

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
    double position = ModuleConstants.FALCON_UNITS_TO_METERS * driveMotor.getSelectedSensorPosition();
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

    // Converts meters per second to rpm
    double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60 
      * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
      
    // Converts rpm to encoder units per 100 milliseconds
    double desiredDriveEncoderUnitsPer100MS = desiredDriveRPM / 600.0 * 2048;

    // Sets the drive motor's speed using the built in pid controller
    driveMotor.set(ControlMode.Velocity, desiredDriveEncoderUnitsPer100MS);

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

  public void periodicFunction() {}
}