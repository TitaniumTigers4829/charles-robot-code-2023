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
    // TODO: get these values from mechanical
    public static final double trackWidth = 0.57785;
    // Distance between centers of right and left wheels on robot
    public static final double wheelBase = 0.57785;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2));

    public static final double voltsS = 0.73394;
    public static final double voltSecondsPerMeterV = 2.4068;
    public static final double voltSecondsSquaredPerMeterA = 0.28749;

    public static final double turningS = 0.77; // LOCKED IN!  -----  old 0.66202
    public static final double turningV = 0.75; //0.75 // 3.0052
    public static final double turningA = 0; // Default to zero
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
  }
  public static final class ModuleConstants {
    public static final double kDriveGearRatio = 7.13;

    // TODO: tune this
    public static final double moduleTurnControllerP = 1;
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
        (10 * wheelCircumferenceMeters) / (kDriveGearRatio * driveFXEncoderCPR);

    public static final TrapezoidProfile.Constraints moduleTurnConstraints =
        new TrapezoidProfile.Constraints(
          maxModuleAngularSpeedRadiansPerSecond,
          maxModuleAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class PathPlannerConstants {

    // Autonomous Period Constants TODO: Tune all of these values
    public static final double autoMaxVelocity = 4.5; // meters/second
    public static final double autoMaxAcceleration = 3.25; // meters/second/second
    public static final double kPXController = 1.25;
    public static final double kPYController = 1.25;
    public static final double kPThetaController = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
}
