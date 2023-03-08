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

  private Constants() {}

  // These are the total encoder units for one revolution
  public static final int FALCON_ENCODER_RESOLUTION = 2048;
  public static final int CANCODER_RESOLUTION = 4096; 

  public static final class DriveConstants {

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.25);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(28.5);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
    );

    public static final double TURNING_S = 0.77;
    public static final double TURNING_V = 0.75;
    public static final double TURNING_A = 0; // Default to zero

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 2;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 12;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 16;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 11;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 9;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 7;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 10;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 13;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 0;
    public static final int FRONT_RIGHT_CANCODER_ID = 2;
    public static final int REAR_LEFT_CANCODER_ID = 1;
    public static final int REAR_RIGHT_CANCODER_ID = 3;

    // In degrees.
    public static final double FRONT_LEFT_ZERO_ANGLE = 47.900390625;
    public static final double FRONT_RIGHT_ZERO_ANGLE = -71.630859375;
    public static final double REAR_LEFT_ZERO_ANGLE = -24.345703125;
    public static final double REAR_RIGHT_ZERO_ANGLE = -88.681640625;

    public static final boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static final boolean REAR_LEFT_CANCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_CANCODER_REVERSED = false;
    
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean REAR_LEFT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean REAR_RIGHT_DRIVE_ENCODER_REVERSED = true;

    public static final double FACEFORWARD_P = 0.015;

    public static final class BalanceConstants {
      // TODO: Tune all these; They are ballpark esitmates
      public static final double BALANCE_P = 0.0472;
      public static final double BALANCE_I = 0;
      public static final double BALANCE_D = 0.003;
      
      public static final double INITIAL_SPEED = 1.4;

      public static final double BALANCE_ERROR_INIT_DEGREES = 14;
      public static final double BALANCE_ERROR_NEAR_BALANCED = 10;
      public static final double BALANCE_ERROR_CONSIDERED_BALANCED = 2.4; // +/- degrees
      public static final double ORIENTATION_ERROR_CONSIDERED_ORIENTED = 2.5; // +/- degrees
      
    }
  }
  
  public static final class ModuleConstants { 

    public static final double DRIVE_GEAR_RATIO = 7.36;

    // TODO: tune this
    public static final double TURN_P = 8.1;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 3 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * Math.PI;

    public static final double DRIVE_F = 0.055;
    public static final double DRIVE_P = 0; // .1
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS =
      WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS_PER_SECOND =
      (10 * WHEEL_CIRCUMFERENCE_METERS) / (DRIVE_GEAR_RATIO * FALCON_ENCODER_RESOLUTION);
    public static final double 
      FALCON_UNITS_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / (DRIVE_GEAR_RATIO * FALCON_ENCODER_RESOLUTION);

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
      );

    public static final String CANIVORE_CAN_BUS_STRING = "Canivore 1";

  }

  public static final class TrajectoryConstants {

    // Autonomous Period Constants TODO: Tune all of these values
    public static final double MAX_SPEED = 1; // meters/second
    public static final double MAX_ACCELERATION = 1; // meters/second/second
    public static final double X_CONTROLLER_P = .5;
    public static final double Y_CONTROLLER_P = .5;
    public static final double THETA_CONTROLLER_P = 0.2;
    public static final double THETA_PROFILED_CONTROLLER_P = 1;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
    // The length of the field in the x direction (left to right)
    public static final double FIELD_LENGTH_METERS = 16.54175;
    // The length of the field in the y direction (top to bottom)
    public static final double FIELD_WIDTH_METERS = 8.0137;
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  
    public static final double BLUE_BALANCE_X_POSITION = 3.885;
    public static final double RED_BALANCE_X_POSITION = 12.635;
    public static final double BALANCE_X_CONTROLLER_P = .3;
    public static final double BALANCE_Y_CONTROLLER_P = .3;
  
    // These are ordered from top left to bottom right from driver perspective
    public static final double[] BLUE_NODE_Y_POSITIONS = 
      {4.9784, 4.4196, 3.8608, 3.302, 2.7432, 2.1844, 1.6256, 1.0668, 0.508};
    public static final double[] RED_NODE_Y_POSITIONS = 
      {0.508, 1.0668, 1.6256, 2.1844, 2.7432, 3.302, 3.8608, 4.4196, 4.9784};
    // Factors in bumper width and wheelbase
    public static final double BLUE_NODE_X_POSITION = 1.5;
    public static final double RED_NODE_Y_POSITION = 15;
    public static final double BLUE_OUTER_WAYPOINT_X = 5.3;
    public static final double RED_OUTER_WAYPOINT_X = 11.2;
    public static final double BLUE_INNER_WAYPOINT_X = 2.5;
    public static final double RED_INNER_WAYPOINT_X = 14;
    public static final double UPPER_WAYPOINT_Y = 4.75;
    public static final double LOWER_WAYPOINT_Y = 0.75;
    public static final Rotation2d BLUE_END_ROTATION = Rotation2d.fromDegrees(180);
    public static final Rotation2d RED_END_ROTATION = Rotation2d.fromDegrees(0);
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

    public static final int LEADER_ROTATION_MOTOR_ID = 0-9;
    public static final int FOLLOWER_ROTATION_MOTOR_ID = 0-9;

    public static final boolean LEADER_ROTATION_MOTOR_INVERTED = false;
    public static final boolean FOLLOWER_ROTATION_MOTOR_INVERTED = true;

    public static final int EXTENSION_MOTOR_ID = 0-9;

    public static final int ROTATION_ENCODER_ID = 0-9;

    public static final double ROTATION_MAX_VELOCITY = 0.0;
    public static final double ROTATION_MAX_ACCELERATION = 0.0;
    public static final double EXTENSION_MAX_VELOCITY = 0.0;
    public static final double EXTENSION_MAX_ACCELERATION = 0.0;
    
    public static final double ROTATION_P = 0-9;
    public static final double ROTATION_I = 0-9;
    public static final double ROTATION_D = 0-9;
    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
      ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCELERATION);
    public static final TrapezoidProfile.Constraints EXTENSION_CONSTRAINTS = new TrapezoidProfile.Constraints(
      EXTENSION_MAX_VELOCITY, EXTENSION_MAX_ACCELERATION);

    public static final double EXTENSION_P = 0-9;
    public static final double EXTENSION_I = 0-9;
    public static final double EXTENSION_D = 0-9;

    public static final double ROTATION_FEED_FORWARD_GAIN = 0-9;
    public static final double ROTATION_ACCELERATION_GAIN = 0-9;
    public static final double ROTATION_VELOCITY_GAIN = 0-9;

    public static final double EXTENSION_FEED_FORWARD_GAIN = 0-9;
    public static final double EXTENSION_ACCELERATION_GAIN = 0-9;
    public static final double EXTENSION_VELOCITY_GAIN = 0-9;

    public static final double EXTENSION_MOTOR_GEAR_RATIO = 1.0 / 5.0;
    public static final double EXTENSION_SPOOL_DIAMETER = Units.inchesToMeters(2.5);
    public static final double MAX_EXTENSION_LENGTH = Units.inchesToMeters(48.48);
    public static final double EXTENSION_ENCODER_OFFSET = 0-9;
  }

  public static final class ClawConstants {

    public static final int SOLENOID_FORWARD = 1; // ID for opening claw
    public static final int SOLENOID_BACKWARD = 0; // ID for closing claw
    public static final int WRIST_MOTOR_ID = 0-9; // ID for the wrist motor
    public static final int LEFT_WHEEL_MOTOR_ID = 0-9; //ID for left claw motor that controls the rollers
    public static final int RIGHT_WHEEL_MOTOR_ID = 0-9; //ID for right claw motor that controls the rollers
    public static final boolean LEFT_WHEEL_MOTOR_INVERTED = false;
    public static final boolean RIGHT_WHEEL_MOTOR_INVERTED = false;
    public static final double WHEELS_MAX_RPM = 0-9;

    public static final double WRIST_FEED_FORWARD_GAIN = 0-9;
    public static final double WRIST_VELOCITY_GAIN = 0-9;
    public static final double WRIST_ACCELERATION_GAIN = 0-9;

    public static final double WRIST_MAX_VELOCITY = 0.0;
    public static final double WRIST_MAX_ACCELERATION = 0.0;

    public static final double WRIST_P = 0-9;
    public static final double WRIST_I = 0-9;
    public static final double WRIST_D = 0-9;
    public static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS = new TrapezoidProfile.Constraints(
      WRIST_MAX_VELOCITY, WRIST_MAX_ACCELERATION);

    public static final int WRIST_LIMIT_SWITCH_PORT = 0-9;

    public static final double MIN_WRIST_ROTATION_DEGREES = 0-9;
    public static final double MIN_WRIST_ROTATION_ENCODER_UNITS = 0-9;
  }

  public static final class JoystickConstants {

    // Ports:
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int BUTTON_BOARD_ID = 1;

    // Axes IDs:
    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int RIGHT_STICK_X = 2;
    public static final int RIGHT_STICK_Y = 3;

    // Button IDs:
    public static final int X_BUTTON_ID = 1;
    public static final int A_BUTTON_ID = 2;
    public static final int B_BUTTON_ID = 3;
    public static final int Y_BUTTON_ID = 4;
    public static final int LEFT_BUMPER_ID = 5;
    public static final int RIGHT_BUMPER_ID = 6;
    public static final int LEFT_TRIGGER_ID = 7;
    public static final int RIGHT_TRIGGER_ID = 8;
    public static final int BACK_BUTTON_ID = 9;
    public static final int START_BUTTON_ID = 10;
    public static final int LEFT_STICK_PRESS_ID = 11;
    public static final int RIGHT_STICK_PRESS_ID = 12;

    public static final int LEFT_DPAD_ID = 270;
    public static final int UP_DPAD_ID = 0;
    public static final int RIGHT_DPAD_ID = 90;
    public static final int DOWN_DPAD_ID = 180;
  }

  public static final class LimelightConstants {
  
    public static final String FRONT_LIMELIGHT_NAME = "limelight-tigers";
    public static final String BACK_LIMELIGHT_NAME = "limelight-jack";

    public static final double[][] APRIL_TAG_POSITIONS = {
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

    public static final double[][] CAMERA_CROP_LOOKUP_TABLE = {
      // TODO: All of these are placeholder values
      // {x position in meters, limelight lower y crop}
      {0, -1},
      {1, -.5},
      {2, -.25},
      {3, 0},
      {4, .25}
    };

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {x position in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 1},
      {1.5, 0.01, 0.01, 1},
      {3, 0.01, 0.10, 6},
      {4.5, 0.2, 0.4, 12},
      {6, 0.7, 1.2, 20}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {x position in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {1.5, 0.01, 0.01, 1},
      {3, 0.01, 0.01, 1},
      {4.5, 0.01, 0.03, 1},
      {6, 0.02, 0.08, 2}
    };

    public static final int DETECTED_FRAMES_FOR_RELIABILITY = 2;
  }

}