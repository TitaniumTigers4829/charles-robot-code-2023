/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                 */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  public static final class DriveConstants {
    public static final double trackWidth = Units.inchesToMeters(29.5);
    // Distance between centers of right and left wheels on robot
    public static final double wheelBase = Units.inchesToMeters(29.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics driveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2)
      );

    public static final double voltsS = 0.73394;
    public static final double voltSecondsPerMeterV = 2.4068;
    public static final double voltSecondsSquaredPerMeterA = 0.28749;

    public static final double turningS = 0.77; // LOCKED IN!  -----  old 0.66202
    public static final double turningV = 0.75; //0.75 // 3.0052
    public static final double turningA = 0; // Default to zero


    public static final double maxAngularSpeedRadiansPerSecond = Math.PI * 2;

    // TODO: find
    public static final double maxSpeedMetersPerSecond = 4.5;

    
    // TODO: get IDs
    public static final int frontLeftDriveMotorPort = 18;
    public static final int rearLeftDriveMotorPort = 6;
    public static final int frontRightDriveMotorPort = 4;
    public static final int rearRightDriveMotorPort = 23;

    public static final int frontLeftTurningMotorPort = 1;
    public static final int rearLeftTurningMotorPort = 7;
    public static final int frontRightTurningMotorPort = 3;
    public static final int rearRightTurningMotorPort = 25;

    public static final int frontLeftTurningEncoderPort = 22;
    public static final int rearLeftTurningEncoderPort = 10;
    public static final int frontRightTurningEncoderPort = 9;
    public static final int rearRightTurningEncoderPort = 8;


    public static final double frontLeftAngleZero = 79.45;
    public static final double rearLeftAngleZero = 121.38;
    public static final double frontRightAngleZero = -104.68;
    public static final double rearRightAngleZero = 23.54;


    public static final boolean frontLeftTurningEncoderReversed = false;
    public static final boolean rearLeftTurningEncoderReversed = false;
    public static final boolean frontRightTurningEncoderReversed = false;
    public static final boolean rearRightTurningEncoderReversed = false;

    public static final boolean frontLeftDriveEncoderReversed = false;
    public static final boolean rearLeftDriveEncoderReversed = false;
    public static final boolean frontRightDriveEncoderReversed = true;
    public static final boolean rearRightDriveEncoderReversed = true;
  }
  
  public static final class ModuleConstants {
    // TODO: get
    // public static final double driveGearRatio = 7.36;
    public static final double driveGearRatio = 7.13;

    // TODO: tune this
    public static final double moduleTurnControllerP = 8.1;
    public static final double moduleTurnControllerI = 0;
    public static final double moduleTurnControllerD = 0;

    public static final double maxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double maxModuleAngularAccelerationRadiansPerSecondSquared = 6 * Math.PI;

    // TODO: tune
    public static final double moduleDriveControllerP = 1; // TUNE
    public static final double moduleDriveControllerI = 0; // DO NOT USE
    public static final double moduleDriveControllerD = 0;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4); // 4 inches
    public static final double wheelCircumferenceMeters =
        wheelDiameterMeters * Math.PI; // C = D * pi
    public static final double drivetoMetersPerSecond =
        (10 * wheelCircumferenceMeters) / (driveGearRatio * driveFXEncoderCPR);

    public static final TrapezoidProfile.Constraints moduleTurnConstraints =
      new TrapezoidProfile.Constraints(
        maxModuleAngularSpeedRadiansPerSecond,
        maxModuleAngularAccelerationRadiansPerSecondSquared
      );
  }

  public static final class PathPlannerConstants {

    // Autonomous Period Constants TODO: Tune all of these values
    public static final double autoMaxVelocity = 4.5; // meters/second
    public static final double autoMaxAcceleration = 3.25; // meters/second/second
    public static final double xControllerP = 1.25;
    public static final double yControllerP = 1.25;
    public static final double thetaControllerP = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
      new TrapezoidProfile.Constraints(maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }
  
  public static final class LEDConstants {

    public static final int LEDPort = 0-9;

    public static final class LEDPatterns {
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
  }

  public static final class ArmConstants {
    
  }

}