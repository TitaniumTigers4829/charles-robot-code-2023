package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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

  public static final double FALCON_ENCODER_RESOLUTION = 2048.0;
  public static final double CANCODER_RESOLUTION = 4096.0; 
  public static final double DEGREES_TO_CANCODER_UNITS = CANCODER_RESOLUTION / 360.0;
  public static final double MIN_FALCON_DEADBAND = 0.001;
  public static final String CANIVORE_CAN_BUS_STRING = "Canivore 1";
  public static final String RIO_CAN_BUS_STRING = "rio";
  public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

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

    public static final double TURN_S = 0.77;
    public static final double TURN_V = 0.75;
    public static final double TURN_A = 0; // Default to zero

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 4;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 7;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int REAR_LEFT_CANCODER_ID = 13;
    public static final int REAR_RIGHT_CANCODER_ID = 14;

    // In degrees.
    public static final double FRONT_LEFT_ZERO_ANGLE = 169.716796875;
    public static final double FRONT_RIGHT_ZERO_ANGLE = -76.46484375;
    public static final double REAR_LEFT_ZERO_ANGLE = 46.58203125;
    public static final double REAR_RIGHT_ZERO_ANGLE = -78.57421875;

    public static final boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static final boolean REAR_LEFT_CANCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_CANCODER_REVERSED = false;
    
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean REAR_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_DRIVE_ENCODER_REVERSED = true;

    public static final double FACEFORWARD_P = 0.015;

    public static final class BalanceConstants {
      // TODO: Tune all these;
      public static final double BALANCE_P = 0.07;
      public static final double BALANCE_I = 0;
      public static final double BALANCE_D = 0;
      
      public static final double ORIENTATION_ERROR_CONSIDERED_ORIENTED = 2.5; // +/- degrees
      public static final double BALANCE_ERROR_CONSIDERED_BALANCED = 2.4; // +/- degrees

      // Might be removed
      public static final double INITIAL_SPEED = 0.7;
      public static final double BALANCE_ERROR_INIT_DEGREES = 10;
      public static final double BALANCE_ERROR_NEAR_BALANCED = 3;
    }
  }
  
  public static final class ModuleConstants { 

    public static final double DRIVE_GEAR_RATIO = 7.36;

    public static final double TURN_P = 7; // 8.1
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 3 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * Math.PI;

    public static final double DRIVE_F = 0.055;
    public static final double DRIVE_P = .1; // .1
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
    public static final double RED_NODE_Y_POSITION = 14.55;
    public static final double BLUE_OUTER_WAYPOINT_X = 5.3;
    public static final double RED_OUTER_WAYPOINT_X = 11.2;
    public static final double BLUE_INNER_WAYPOINT_X = 2.5;
    public static final double RED_INNER_WAYPOINT_X = 14;
    public static final double UPPER_WAYPOINT_Y = 4.75;
    public static final double LOWER_WAYPOINT_Y = 0.75;
    public static final Rotation2d BLUE_END_ROTATION = Rotation2d.fromDegrees(180);
    public static final Rotation2d RED_END_ROTATION = Rotation2d.fromDegrees(0);

    // Auto Trajectory Names
    public static final String TWO_CONE_BALANCE_AUTO_FIRST = "Two Cone Balance First";
    public static final String TWO_CONE_BALANCE_AUTO_SECOND = "Two Cone Balance Second";
    public static final String TWO_CONE_BALANCE_AUTO_THIRD = "Two Cone Balance Third";
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
    // speed to move the arm before doing the real movement
    public static final double ARM_MOVE_SPEED_BEFORE_REAL_MOVE = 0.15;

    // physics stuff
    public static final double ARM_WEIGHT_NEWTONS = 9.8 * 25;
    public static final double ARM_AXIS_OF_ROTATION_RADIUS = Units.inchesToMeters(2.1);

    // unit conversions
    public static final double ARM_ROTATION_GEAR_RATIO = 1.0 / 192.0;
    public static final double ARM_ENCODER_UNITS_TO_DEGREES = (360.0 / Constants.CANCODER_RESOLUTION) * ARM_ROTATION_GEAR_RATIO;
    public static final double ARM_DEGREES_TO_CANCODER_UNITS = (Constants.CANCODER_RESOLUTION / 360.0) / ARM_ROTATION_GEAR_RATIO;
    public static final double ARM_DEGREES_TO_FALCON_UNITS = Constants.FALCON_ENCODER_RESOLUTION / 360.0;
    public static final double ARM_CANCODER_DEGREES_TO_CANCODER_UNITS = Constants.CANCODER_RESOLUTION / 360.0;
    public static final double ARM_ROTATION_MOTOR_TO_DEGREES = Constants.FALCON_ENCODER_RESOLUTION / 360.0 * ARM_ROTATION_GEAR_RATIO;

    // motor IDs and config
    public static final int LEADER_ROTATION_MOTOR_ID = 9;
    public static final int FOLLOWER_ROTATION_MOTOR_ID = 10;
    public static final int ROTATION_ENCODER_ID = 15;

    public static final double ROTATION_ENCODER_OFFSET = 95.009765625;

    public static final boolean LEADER_ROTATION_MOTOR_INVERTED = false;
    public static final boolean FOLLOWER_ROTATION_MOTOR_INVERTED = true;
    
    // PID constants
    public static final double ROTATION_FEED_FORWARD_CONSTANT = -.00025;
    public static final double ROTATION_P = 3.75;
    public static final double ROTATION_I = 0;
    public static final double ROTATION_D = 0;

    //  max velocity stuff
    public static final double ROTATION_MAX_VELOCITY_ENCODER_UNITS = 180;
    public static final double ROTATION_MAX_ACCELERATION_ENCODER_UNITS = 180;
    public static final int ROTATION_SMOOTHING = 1;
    // public static final double ROTATION_TOLERANCE_DEGREES= .5;
    public static final double ROTATION_TOLERANCE_DEGREES = 0;
    public static final double ROTATION_TOLERANCE_ENCODER_UNITS = ROTATION_TOLERANCE_DEGREES * DEGREES_TO_CANCODER_UNITS;
    public static final double MAX_ROTATION_ENCODER_UNITS = 305 * DEGREES_TO_CANCODER_UNITS;
    public static final double MIN_ROTATION_ENCODER_UNITS = 55 * DEGREES_TO_CANCODER_UNITS;

    // unit conversions
    public static final double EXTENSION_MOTOR_GEAR_RATIO = 16.0;
    public static final double EXTENSION_SPOOL_DIAMETER = Units.inchesToMeters(2.5);
    public static final double MAX_EXTENSION_LENGTH = Units.inchesToMeters(48.48);
    public static final double EXTENSION_MOTOR_POS_TO_METERS = EXTENSION_SPOOL_DIAMETER / (Constants.FALCON_ENCODER_RESOLUTION * EXTENSION_MOTOR_GEAR_RATIO) * Math.PI;
    public static final double EXTENSION_METERS_TO_MOTOR_POS = -1.0 / EXTENSION_MOTOR_POS_TO_METERS;

    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
      ROTATION_MAX_VELOCITY_ENCODER_UNITS, ROTATION_MAX_ACCELERATION_ENCODER_UNITS);

    public static final double EXTENSION_ACCELERATION_GAIN = 0;
    public static final double EXTENSION_VELOCITY_GAIN = 0.2;
    public static final double EXTENSION_P = -8;
    public static final double EXTENSION_I = 0;
    public static final double EXTENSION_D = 0;
    public static final double EXTENSION_F = 0;

    // pneumatic lock stuff
    public static final int EXTENSION_LOCK_ENGAGED_ID = 2;
    public static final int EXTENSION_LOCK_DISENGAGED_ID = 3;

    // max/min output
    public static final double EXTENSION_MOTOR_MIN_OUTPUT = -.2;
    public static final double EXTENSION_MOTOR_MAX_OUTPUT = .75;
    public static final double EXTENSION_MAX_VELOCITY = 1;
    public static final double EXTENSION_MAX_ACCELERATION = 2.5;

    public static final int EXTENSION_MOTOR_ID = 16;
    public static final boolean EXTENSION_MOTOR_INVERTED = true;

    // acceptable error
    public static final double EXTENSION_ACCEPTABLE_ERROR = 0.025; 
    public static final TrapezoidProfile.Constraints EXTENSION_CONSTRAINTS = new TrapezoidProfile.Constraints(
      EXTENSION_MAX_VELOCITY, EXTENSION_MAX_ACCELERATION);

    public static final int STALLING_VELOCITY = 150;
    public static final int TICKS_BEFORE_STALL = 10;

    public static final double[][] CENTER_OF_MASS_LOOKUP_TABLE = {
      //{extension (meters), distance from pivot point to COM}
      {0, Units.inchesToMeters(1.024)},
      {.1, Units.inchesToMeters(2.9)},
      {.2, Units.inchesToMeters(4.5)},
      {.4, Units.inchesToMeters(8.4)},
      {.6, Units.inchesToMeters(12)},
      {.75, Units.inchesToMeters(16)},
      {1.05, Units.inchesToMeters(20)},
      {1.25, Units.inchesToMeters(34)},
    };

    public static final double[][] ARM_EXTENSION_CORRECTION_LOOKUP_TABLE = {
    //{arm rotation (degrees), voltage}
      {}
    };

    // TODO: find out
    public static final double PICKUP_FROM_GROUND_EXTENSION = 0.6;
    public static final double PICKUP_FROM_GROUND_ROTATION = 290;

    public static final double PLACE_HIGH_EXTENSION = 0.95;
    public static final double PLACE_HIGH_ROTATION = 242;

    public static double ROTATION_ACCEPTABLE_ERROR;
  }

  public static final class ClawConstants {
    public static final double WRIST_POS_TO_DEG = (360.0 / Constants.FALCON_ENCODER_RESOLUTION) / (76.0 / 20.0);
    public static final double DEG_TO_WRIST_POS = (Constants.FALCON_ENCODER_RESOLUTION / 360.0) * (76.0 / 20.0);

    public static final int SOLENOID_FORWARD = 0;
    public static final int SOLENOID_BACKWARD = 1;

    public static final int WRIST_MOTOR_ID = 17;
    public static final int INTAKE_MOTOR_ID = 18;

    public static final boolean INTAKE_MOTOR_INVERTED = false;
    public static final boolean WRIST_MOTOR_INVERTED = true;

    public static final double WRIST_MAX_RADIANS_PER_SECOND = 180;
    public static final double WRIST_MAX_RADIANS_PER_SECOND_SQUARED = 180.0 / 24;

    public static final double WRIST_F = 0;
    public static final double WRIST_P = 2;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0;
    public static final double WRIST_I_ZONE = 0;

    public static final double WRIST_MAX_VELOCITY_ENCODER_UNITS = 90 * DEG_TO_WRIST_POS;
    public static final double WRIST_MAX_ACCELERATION_ENCODER_UNITS = 120 * DEG_TO_WRIST_POS;
    public static final int WRIST_SMOOTHING = 0;
    public static final double WRIST_TOLERANCE = 0 * DEG_TO_WRIST_POS;
    
    public static final double INTAKE_F = 0;
    public static final double INTAKE_P = 0;
    public static final double INTAKE_I = 0;
    public static final double INTAKE_D = 0;

    public static final double MIN_WRIST_ROTATION_ENCODER_UNITS = 0 * DEG_TO_WRIST_POS;
    public static final double MAX_WRIST_ROTATION_ENCODER_UNITS = 180 * DEG_TO_WRIST_POS;
  }

  public static final class JoystickConstants {

    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
    public static final int BUTTON_BOARD_1_ID = 2;
    public static final int BUTTON_BOARD_2_ID = 3;

    public static final int DRIVER_LEFT_STICK_X = 0;
    public static final int DRIVER_LEFT_STICK_Y = 1;
    public static final int DRIVER_RIGHT_STICK_X = 4;
    public static final int DRIVER_RIGHT_STICK_Y = 5;
    public static final int DRIVER_X_BUTTON_ID = 3;
    public static final int DRIVER_A_BUTTON_ID = 1;
    public static final int DRIVER_B_BUTTON_ID = 2;
    public static final int DRIVER_Y_BUTTON_ID = 4;
    public static final int DRIVER_LEFT_BUMPER_ID = 5;
    public static final int DRIVER_RIGHT_BUMPER_ID = 6;
    public static final int DRIVER_LEFT_TRIGGER_ID = 2;
    public static final int DRIVER_RIGHT_TRIGGER_ID = 3;
    public static final int DRIVER_BACK_BUTTON_ID = 7;
    public static final int DRIVER_START_BUTTON_ID = 8;
    public static final int DRIVER_LEFT_STICK_PRESS_ID = 9;
    public static final int DRIVER_RIGHT_STICK_PRESS_ID = 10;

    public static final int OPERATOR_LEFT_STICK_X = 0;
    public static final int OPERATOR_LEFT_STICK_Y = 1;
    public static final int OPERATOR_RIGHT_STICK_X = 4;
    public static final int OPERATOR_RIGHT_STICK_Y = 5;

    public static final int OPERATOR_X_BUTTON_ID = 3;
    public static final int OPERATOR_A_BUTTON_ID = 1;
    public static final int OPERATOR_B_BUTTON_ID = 2;
    public static final int OPERATOR_Y_BUTTON_ID = 4;
    public static final int OPERATOR_LEFT_BUMPER_ID = 5;
    public static final int OPERATOR_RIGHT_BUMPER_ID = 6;
    public static final int OPERATOR_LEFT_TRIGGER_ID = 2;
    public static final int OPERATOR_RIGHT_TRIGGER_ID = 3;
    public static final int OPERATOR_BACK_BUTTON_ID = 7;
    public static final int OPERATOR_START_BUTTON_ID = 8;
    public static final int OPERATOR_LEFT_STICK_PRESS_ID = 9;
    public static final int OPERATOR_RIGHT_STICK_PRESS_ID = 10;

    public static final int LEFT_DPAD_ID = 270;
    public static final int UP_DPAD_ID = 0;
    public static final int RIGHT_DPAD_ID = 90;
    public static final int DOWN_DPAD_ID = 180;

    // Top Six Buttons

    // Z Axis
    public static final int BIG_BUTTON_1 = -1;
    public static final int BIG_BUTTON_2 = 1;
    // X Axis
    public static final int BIG_BUTTON_3 = -1;
    public static final int BIG_BUTTON_4 = 1;
    // Y Axis
    public static final int BIG_BUTTON_5 = -1;
    public static final int BIG_BUTTON_6 = 1;

    // Joystick 1 Autoplace Buttons
    public static final int BUTTON_1 = 1;
    public static final int BUTTON_2 = 2;
    public static final int BUTTON_3 = 3;
    public static final int BUTTON_4 = 4;
    public static final int BUTTON_5 = 5;
    public static final int BUTTON_6 = 6;
    public static final int BUTTON_7 = 7;
    public static final int BUTTON_8 = 8;
    public static final int BUTTON_9 = 9;
    public static final int BUTTON_10 = 10;
    public static final int BUTTON_11 = 11;
    public static final int BUTTON_12 = 12;
    public static final int BUTTON_13 = 0; // POV Joystick 1
    public static final int BUTTON_14 = 180; // POV Joystick 1
    public static final int BUTTON_15 = 270; // POV Joystick 1
    public static final int BUTTON_16 = 90; // POV Joystick 1

    // Joystick 2 Autoplace Buttons
    public static final int BUTTON_17 = 1;
    public static final int BUTTON_18 = 2;
    public static final int BUTTON_19 = 3;
    public static final int BUTTON_20 = 4;
    public static final int BUTTON_21 = 5;
    public static final int BUTTON_22 = 6;
    public static final int BUTTON_23 = 7;
    public static final int BUTTON_24 = 8;
    public static final int BUTTON_25 = 9;
    public static final int BUTTON_26 = 10;
    public static final int BUTTON_27 = 11;
    
  }

  public static final class LimelightConstants {
  
    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    public static final String BACK_LIMELIGHT_NAME = "limelight-back";

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
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 10},
      {1.5, 0.01, 0.01, 10},
      {3, 0.145, 1.20, 30},
      {4.5, 0.5, 4.0, 90},
      {6, 0.75, 8.0, 180}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {1.5, 0.01, 0.01, 5},
      {3, 0.01, 0.01, 5},
      {4.5, 0.01, 0.03, 10},
      {6, 0.02, 0.08, 20}
    };
  }

}