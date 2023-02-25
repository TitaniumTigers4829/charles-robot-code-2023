// Some of this code was copied from Team 7028 - Binary Battalion's swerve-test repository
// https://github.com/STMARobotics/swerve-test/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java

package frc.robot.subsystems.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrajectoryConstants;

public class VisionSubsystemImpl extends SubsystemBase implements VisionSubsystem {
  
  private final NetworkTable networkTable; 
  private final NetworkTableEntry botPoseNetworkTableEntry;
  private final NetworkTableEntry jsonDumpNetworkTableEntry;
  private final NetworkTableEntry cameraCropNetworkTableEntry;
  
  private final ObjectMapper mapper = new ObjectMapper();

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
    double[] botPose = botPoseNetworkTableEntry.getDoubleArray(new double[]{});
    if (botPose.length == 0) {
      return false;
    } else {
      return !(botPose[0] == 0 && botPose[1] == 0 && botPose[2] == 0);
    }
  }

  @Override
  public Pose2d getPoseFromAprilTags() {
    double[] botPose = botPoseNetworkTableEntry.getDoubleArray(new double[]{});
    double robotX = botPose[0] + TrajectoryConstants.fieldLengthMeters / 2;
    double robotY = botPose[1] + TrajectoryConstants.fieldWidthMeters / 2;
    Rotation2d robotRotation = Rotation2d.fromDegrees(botPose[5]);
    return new Pose2d(robotX, robotY, robotRotation);
  }

  @Override
  public long getTimeStampSeconds() {
    String jsonDump = jsonDumpNetworkTableEntry.getString("{}");  

    try {
      JsonNode jsonNodeData = mapper.readTree(jsonDump);
      // Converts milliseconds to seconds
      return jsonNodeData.path("Results").path("ts").asLong() / 1000;
    } catch (JsonProcessingException e) {
      SmartDashboard.putString("Json Parsing Error", e.getLocalizedMessage());
    }

    return 0L;
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
    cameraCropNetworkTableEntry.setDoubleArray(cropValues[0]);
  }

}