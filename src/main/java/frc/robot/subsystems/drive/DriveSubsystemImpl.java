package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.extras.SmartDashboardLogger;

public class DriveSubsystemImpl extends SubsystemBase implements DriveSubsystem {

  // This will stay the same throughout the match. These values are harder to test for and tune, so assume this guess is right.
  private final Vector<N3> stateStandardDeviations = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1));
  // This will be changed throughout the match depending on how confident we are that the limelight is right.
  private final Vector<N3> visionMeasurementStandardDeviations = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(50));

  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule rearLeftSwerveModule;
  private final SwerveModule rearRightSwerveModule;

  private final Gyro gyro;
  private final SwerveDrivePoseEstimator odometry;

  private double gyroOffset = 0;
  // private float rollOffset = 0;
  // private float pitchOffset = 0;

  private int selectedNode = 1;
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystemImpl() {
    frontLeftSwerveModule = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
      DriveConstants.FRONT_LEFT_CANCODER_ID,
      DriveConstants.FRONT_LEFT_ZERO_ANGLE,
      DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      "FL"
    );
    
    frontRightSwerveModule = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_CANCODER_ID,
      DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
      DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      "FR"
    );
    
    rearLeftSwerveModule = new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
      DriveConstants.REAR_LEFT_CANCODER_ID,
      DriveConstants.REAR_LEFT_ZERO_ANGLE,
      DriveConstants.REAR_LEFT_CANCODER_REVERSED,
      DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
      "RL"
    );
    
    rearRightSwerveModule = new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
      DriveConstants.REAR_RIGHT_CANCODER_ID,
      DriveConstants.REAR_RIGHT_ZERO_ANGLE,
      DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
      DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
      "RR"
    );

    gyro = new AHRS(SPI.Port.kMXP);
  
    odometry = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match
      stateStandardDeviations,
      visionMeasurementStandardDeviations
    );

    SmartDashboardLogger.infoNumber("Selected Node", selectedNode);
  }

  @SuppressWarnings("ParameterName")
  @Override
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    // SmartDashboard.putBoolean("isFieldRelative", fieldRelative);
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getFieldRelativeRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public double getHeading() {
    // return (-gyro.getAngle() + this.gyroOffset) % 360;
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  @Override
  public double getRoll() {
    // return (-gyro.getRoll() + this.rollOffset) % 360;
    return 0;
  }

  @Override
  public double getPitch() {
    // return (-gyro.getPitch() + this.pitchOffset) % 360;
    return 0;
  }

  @Override
  public double getBalanceError() {
    double roll = getRoll();
    double pitch = getPitch();
    double yaw = getHeading();
    return (pitch * Math.cos(yaw)) + (roll * Math.sin(yaw));
  }

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public Rotation2d getFieldRelativeRotation2d() {
    // Because the field isn't vertically symmetrical, we have the pose coordinates always start from the bottom left
    return Rotation2d.fromDegrees((getHeading() + (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)) % 360);
  }

  @Override
  public void setGyroOffset(double gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  @Override
  public void zeroHeading() {
    gyroOffset = (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180) % 360;
    gyro.reset();
  }

  @Override
  public void zeroPitchAndRoll() {
    // pitchOffset = gyro.getPitch();
    // rollOffset = gyro.getRoll();
  }

  @Override
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Override
  public void addPoseEstimatorSwerveMeasurement() {
    odometry.update(
      getRotation2d(),
      getModulePositions()
    );
  }
  
  @Override
  public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    odometry.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  @Override
  public void resetOdometryAndRotation(Pose2d pose, double angle) {
    zeroHeading();
    setGyroOffset(angle);
    odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  @Override
  public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }
  
  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };

    return swerveModulePositions;
  }

  @Override
  public int getSelectedNode() {
    return selectedNode;
  }
  
  @Override
  public void setSelectedNode(int nodeID) {
    selectedNode = nodeID;
    SmartDashboardLogger.infoNumber("Selected Node", selectedNode);
  }

  @Override
  public void periodic() {
    Pose2d estimatedPose = odometry.getEstimatedPosition();
    SmartDashboardLogger.infoString("Estimated pose", estimatedPose.toString());
    frontLeftSwerveModule.periodicFunction();
    frontRightSwerveModule.periodicFunction();
    rearLeftSwerveModule.periodicFunction();
    rearRightSwerveModule.periodicFunction();
  }

}