package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.extras.SmartDashboardLogger;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ManualArm extends CommandBase {

  private ArmSubsystem armSubsystem;
  private DoubleSupplier armRotationSpeed;
  private DoubleSupplier armExtensionSpeed;
  private BooleanSupplier isManual;

  public ManualArm(ArmSubsystem armSubsystem, DoubleSupplier armRotationSpeed, DoubleSupplier armExtensionSpeed, BooleanSupplier isManual) {
    this.armSubsystem = armSubsystem;
    this.armRotationSpeed = armRotationSpeed;
    this.armExtensionSpeed = armExtensionSpeed;
    this.isManual = isManual;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // if (armSubsystem.isManualControl()) {
    if (isManual.getAsBoolean()) {
        SmartDashboardLogger.infoBoolean("running", true);
      if (Math.abs(armRotationSpeed.getAsDouble()) > 0.1) {
        armSubsystem.setRotationSpeed(armRotationSpeed.getAsDouble());
      } else {
        armSubsystem.setRotationSpeed(0);
      }

      if (Math.abs(armExtensionSpeed.getAsDouble()) > 0.1) {
          armSubsystem.setExtensionSpeed(armExtensionSpeed.getAsDouble());
      } else {
        armSubsystem.setExtensionSpeed(ArmConstants.IDLE_EXTENSION_OUTPUT);
      }
    } else { 
      SmartDashboardLogger.infoBoolean("running", false);

    }
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
