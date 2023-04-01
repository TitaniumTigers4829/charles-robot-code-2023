package frc.robot.extras;

public class NodeRegistry {

  private static int selectedNode = 0;

  private NodeRegistry() {}

  public static void setSelectedNode(int nodeID) {
    selectedNode = nodeID;
  }

  public static int getSelectedNode() {
    return selectedNode;
  }
}