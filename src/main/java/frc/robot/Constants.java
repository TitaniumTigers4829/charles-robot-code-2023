/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                 */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int driveFXEncoderCPR = 2048;
  public static final int turningCANcoderCPR = 4096;

  private Constants() {}

  public static final class DriveConstants {
    // Distance between centers of right and left wheels on robot
    public static final double trackWidth = Units.inchesToMeters(22.25);
    // Distance between front and back wheels on robot
    public static final double wheelBase = Units.inchesToMeters(28.5);
    public static final SwerveDriveKinematics driveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2), // Front Left
        new Translation2d(wheelBase / 2, -trackWidth / 2), // Front Right
        new Translation2d(-wheelBase / 2, trackWidth / 2), // Rear Left
        new Translation2d(-wheelBase / 2, -trackWidth / 2) // Rear Right
      );

    public static final double voltsS = 0.73394;
    public static final double voltSecondsPerMeterV = 2.4068;
    public static final double voltSecondsSquaredPerMeterA = 0.28749;

    public static final double turningS = 0.77; // LOCKED IN!  -----  old 0.66202
    public static final double turningV = 0.75; //0.75 // 3.0052
    public static final double turningA = 0; // Default to zero


    public static final double maxAngularSpeedRadiansPerSecond = Math.PI * 2;

    public static final double joystickMaxSpeedMetersPerSecondLimit = 4;

    public static final int frontLeftDriveMotorPort = 12;
    public static final int frontRightDriveMotorPort = 16;
    public static final int rearLeftDriveMotorPort = 11;
    public static final int rearRightDriveMotorPort = 9;

    public static final int frontLeftTurningMotorPort = 7;
    public static final int frontRightTurningMotorPort = 10;
    public static final int rearLeftTurningMotorPort = 13;
    public static final int rearRightTurningMotorPort = 8;

    public static final int frontLeftTurningEncoderPort = 0;
    public static final int frontRightTurningEncoderPort = 2;
    public static final int rearLeftTurningEncoderPort = 1;
    public static final int rearRightTurningEncoderPort = 3;

    // In degrees.
    public static final double frontLeftAngleZero = 47.900390625; //-42.099609375
    public static final double frontRightAngleZero = -71.630859375;
    public static final double rearLeftAngleZero = -24.345703125; //65.654296875
    public static final double rearRightAngleZero = -88.681640625;

    public static final boolean frontLeftTurningEncoderReversed = false;
    public static final boolean frontRightTurningEncoderReversed = false;
    public static final boolean rearLeftTurningEncoderReversed = false;
    public static final boolean rearRightTurningEncoderReversed = false;
    
    public static final boolean frontLeftDriveEncoderReversed = true;
    public static final boolean frontRightDriveEncoderReversed = true;
    public static final boolean rearLeftDriveEncoderReversed = true;
    public static final boolean rearRightDriveEncoderReversed = true;
  }
  
  public static final class ModuleConstants {

    public static final double driveGearRatio = 7.36;

    // TODO: tune this
    public static final double moduleTurnControllerP = 8.1;
    public static final double moduleTurnControllerI = 0;
    public static final double moduleTurnControllerD = 0;

    public static final double maxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double maxModuleAngularAccelerationRadiansPerSecondSquared = 6 * Math.PI;

    public static final double moduleDriveControllerF = 0.055;
    public static final double moduleDriveControllerP = 0; // .1
    public static final double moduleDriveControllerI = 0;
    public static final double moduleDriveControllerD = 0;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4); // 4 inches
    public static final double wheelCircumferenceMeters =
        wheelDiameterMeters * Math.PI; // C = D * pi
    public static final double drivetoMetersPerSecond =
        (10 * wheelCircumferenceMeters) / (driveGearRatio * driveFXEncoderCPR);
    public static final double falconToMeters = wheelCircumferenceMeters / (driveGearRatio * 2048);

    public static final TrapezoidProfile.Constraints moduleTurnConstraints =
      new TrapezoidProfile.Constraints(
        maxModuleAngularSpeedRadiansPerSecond,
        maxModuleAngularAccelerationRadiansPerSecondSquared
      );

    public static final String canivoreCanBusString = "Canivore 1";

  }

  public static final class TrajectoryConstants {

    // Autonomous Period Constants TODO: Tune all of these values
    public static final double autoMaxVelocity = 1; // meters/second
    public static final double autoMaxAcceleration = 1; // meters/second/second
    public static final double xControllerP = .5;
    public static final double yControllerP = .5;
    public static final double thetaControllerP = 0.2;
    public static final double thetaProfiledControllerP = 1;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;
    // The length of the field in the x direction (left to right)
    public static final double fieldLengthMeters = 16.54175;
    // The length of the field in the y direction (top to bottom)
    public static final double fieldWidthMeters = 8.0137;
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
      new TrapezoidProfile.Constraints(maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  
    public static final double blueBalanceXPosition = 3.885;
    public static final double redBalanceXPosition = 12.635;
    public static final double balanceXControllerP = .3;
    public static final double balanceYControllerP = .3;
  
    // These are ordered from top left to bottom right from driver perspective
    public static final double[] blueNodeYPositions = 
      {4.9784, 4.4196, 3.8608, 3.302, 2.7432, 2.1844, 1.6256, 1.0668, 0.508};
    public static final double[] redNodeYPositions = 
      {0.508, 1.0668, 1.6256, 2.1844, 2.7432, 3.302, 3.8608, 4.4196, 4.9784};
    // Factors in bumper width and wheelbase
    public static final double blueNodeXPosition = 1.5;
    public static final double redNodeXPosition = 15;
    public static final double blueOuterWaypointX = 5.3;
    public static final double redOuterWaypointX = 11.2;
    public static final double blueInnerWaypointX = 2.5;
    public static final double redInnerWaypointX = 14;
    public static final double upperWaypointY = 4.75;
    public static final double lowerWaypointY = 0.75;
    public static final Rotation2d blueEndRotation = Rotation2d.fromDegrees(180);
    public static final Rotation2d redEndRotation = Rotation2d.fromDegrees(0);
  }
  
  public static final class LEDConstants {

    public static final int LEDPort = 0-9;

    public static final class SparkMaxConstants {
      // This subclass contains the constant values for the LED patterns.
      public static final double RAINBOW = -0.99;

      public static final double SHOT_RED = -0.85;
      public static final double SHOT_BLUE = -0.83;
      public static final double SHOT_WHITE = -0.81;

      public static final double FIRE = -0.57;
      public static final double RAINBOW_WAVE = -0.45;
      public static final double OCEAN = -0.41;

      public static final double BOUNCE_RED = -0.35;
      public static final double BOUNCE_GRAY = -0.33;

      public static final double HEARTBEAT_RED = -0.25;
      public static final double HEARTBEAT_GRAY = -0.19;

      public static final double STROBE_RED = -0.11;
      public static final double STROBE_BLUE = -0.09;
      public static final double STROBE_GOLD = -0.07;
      public static final double STROBE_WHITE = -0.05;

      public static final double MAGENTA = 0.57;
      public static final double DARK_RED = 0.59;
      public static final double RED = 0.61;
      public static final double VERMILION = 0.63;
      public static final double ORANGE = 0.65;
      public static final double GOLD = 0.67;
      public static final double YELLOW = 0.69;

      public static final double LAWN_GREEN = 0.71;
      public static final double LIME = 0.73;
      public static final double DARK_GREEN = 0.75;
      public static final double GREEN = 0.77;
      public static final double CYAN = 0.79;

      public static final double AQUA = 0.81;
      public static final double SKY_BLUE = 0.83;
      public static final double DARK_BLUE = 0.85;
      public static final double BLUE = 0.87;

      public static final double INDIGO = 0.89;
      public static final double PURPLE = 0.91;

      public static final double WHITE = 0.93;
      public static final double GRAY = 0.95;
      public static final double DARK_GRAY = 0.97;
      public static final double BLACK = 0.99;    }
    public enum LEDProcess {
      ALLIANCE_COLOR (0), // Values are null on purpose.
      DEFAULT (0),
      RAINBOW (SparkMaxConstants.RAINBOW),
      RED_ALLIANCE (SparkMaxConstants.FIRE),
      BLUE_ALLIANCE (SparkMaxConstants.OCEAN),
      INTAKE (SparkMaxConstants.MAGENTA),
      SCORING (SparkMaxConstants.YELLOW),
      BALANCE (SparkMaxConstants.CYAN),
      OFF (SparkMaxConstants.BLACK),
      AUTONOMOUS (SparkMaxConstants.RAINBOW_WAVE),
      LINE_UP (SparkMaxConstants.WHITE);

      private final double sparkMaxValue;
      LEDProcess(double sparkMaxValue) {
        this.sparkMaxValue = sparkMaxValue;
      }
      public double getSparkMaxValue() { return sparkMaxValue; }

    }
  }

  public static final class ArmConstants {
    public static final int rotationMotorID = 0-9;
    public static final int extensionMotorID = 0-9;

    public static final int rotationEncoderID = 0-9;
    public static final int extensionEncoderID = 0-9;

    public static final int extensionLimitSwitchID = 0-9;

    public static final int extensionMotorMaxOutput = 1;
    public static final int extensionMotorMinOutput = 0;

    public static final double rotationFeedForwardGain = 0-9;
    public static final double rotationAccelerationGain = 0-9;
    public static final double rotationVelocityGain = 0-9;

    public static final double extensionFeedForwardGain = 0-9;
    public static final double extensionAccelerationGain = 0-9;
    public static final double extensionVelocityGain = 0-9;
    
    public static final double rotationPValue = 0-9;
    public static final double rotationIValue = 0-9;
    public static final double rotationDValue = 0-9;

    public static final double extensionPValue = 0-9;
    public static final double extensionIValue = 0-9;
    public static final double extensionDValue = 0-9;
  }

  public static final class ClawConstants {

    public static final int solenoidForward = 1; // ID for opening claw
    public static final int solenoidBackward = 0; // ID for closing claw
    public static final int wristMotorID = 0-9; // ID for the wrist motor
    public static final int leftClawMotorID = 0-9; //ID for left claw motor that controls the rollers
    public static final int rightClawMotorID = 0-9; //ID for right claw motor that controls the rollers

  }

  public static final class JoystickConstants {

    // Ports:
    public static final int driverJoystickID = 0;
    public static final int buttonBoardID = 1;

    // Axes IDs:
    public static final int leftStickX = 0;
    public static final int leftStickY = 1;
    public static final int rightStickX = 2;
    public static final int rightStickY = 3;

    // Button IDs:
    public static final int xButtonID = 1;
    public static final int aButtonID = 2;
    public static final int bButtonID = 3;
    public static final int yButtonID = 4;
    public static final int leftBumperID = 5;
    public static final int rightBumperID = 6;
    public static final int leftTriggerID = 7;
    public static final int rightTriggerID = 8;
    public static final int backButtonID = 9;
    public static final int startButtonID = 10;
    public static final int leftStickPressID = 11;
    public static final int rightStickPressID = 12;

    public static final int leftDPadID = 270;
    public static final int upDPadID = 0;
    public static final int rightDPadID = 90;
    public static final int downDPadID = 180;
  }

  public static final class LimelightConstants {
  
    public static final String frontLimelightName = "limelight-tigers";
    public static final String backLimelightName = "limelight-jack";

    public static final double limelightHeight = Units.inchesToMeters(40);
    public static final double limelightPitchDegrees = -30;

    public static final double[][] aprilTagPositions = {
      // { x, y, z}
      {Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22)}, // 1
      {Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22)}, // 2
      {Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22)}, // 3
      {Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38)}, // 4
      {Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38)}, // 5
      {Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22)}, // 6
      {Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22)}, // 7
      {Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22)} // 8
    };

    public static final double[][] cameraCropLookupTable = {
      // TODO: All of these are placeholder values
      // {x position in meters, limelight lower y crop}
      {0, -1},
      {1, -.5},
      {2, -.25},
      {3, 0},
      {4, .25}
    };

    public static final double[][] oneAprilTagLookupTable = {
      // {x position in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 1},
      {1.5, 0.01, 0.01, 1},
      {3, 0.01, 0.10, 6},
      {4.5, 0.2, 0.4, 12},
      {6, 0.7, 1.2, 20}
    };

    public static final double[][] twoAprilTagLookupTable = {
      // {x position in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {1.5, 0.01, 0.01, 1},
      {3, 0.01, 0.01, 1},
      {4.5, 0.01, 0.03, 1},
      {6, 0.02, 0.08, 2}
    };

    public static final int detectedFramesForReliability = 2;
  }

}