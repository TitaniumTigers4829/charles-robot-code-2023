// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwitchLimelightPipeline extends CommandBase {

  private int pipelineIndex;

  /** Switches the active pipeline of the limelight.
   * @param pipelineIndex The index from 0-9 of the pipeline.
   */
  public SwitchLimelightPipeline(int pipelineIndex) {
    this.pipelineIndex = pipelineIndex;
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight-tigers").getEntry("pipeline").setNumber(pipelineIndex);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
