// Some of this code was copied from Team 7028 - Binary Battalion's swerve-test repository
// https://github.com/STMARobotics/swerve-test/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class VisionSubsystemImpl extends SubsystemBase implements VisionSubsystem {
  
  private final NetworkTable networkTable; 
  private final NetworkTableEntry botPoseNetworkTableEntry;
  private final NetworkTableEntry jsonDumpNetworkTableEntry;
  private final NetworkTableEntry cameraCropNetworkTableEntry;

  public VisionSubsystemImpl() {    
    networkTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.frontLimelightName);
    botPoseNetworkTableEntry = networkTable.getEntry("botpose");
    jsonDumpNetworkTableEntry = networkTable.getEntry("json");
    cameraCropNetworkTableEntry = networkTable.getEntry("crop");
  }

  @Override
  public void periodic() {}

  @Override
  public boolean canSeeAprilTags() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Pose2d getPoseFromAprilTags() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public long getTimeStampInMilleseconds() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean canSeeCube() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Translation2d getClosestCubePosition() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public boolean canSeeCone() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Translation2d getClosestConePosition() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void cropLimelights(double[][] cropValues) {
    // TODO Auto-generated method stub
  
  }



}