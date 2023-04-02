package frc.robot.extras;

public class NodeAndModeRegistry {

  private static int selectedNode = 19;
  private static boolean isConeMode = false;

  private NodeAndModeRegistry() {}

  public static void init() {
    SmartDashboardLogger.infoNumber("Selected Node", selectedNode);
  }

  public static void setSelectedNode(int nodeID) {
    selectedNode = nodeID;
    SmartDashboardLogger.infoNumber("Selected Node", nodeID);
    SmarterDashboardRegistry.setNode(nodeID);
  }

  public static int getSelectedNode() {
    return selectedNode;
  }

  public static boolean isConeMode() {
    return isConeMode;
  }
  public static void toggleMode() {
    isConeMode = !isConeMode;
    SmarterDashboardRegistry.setIsConeMode(isConeMode);
  }
  public static void setIsConeMode(boolean isConeMode_) {
    isConeMode = isConeMode_;
    SmarterDashboardRegistry.setIsConeMode(isConeMode_);
  }
}