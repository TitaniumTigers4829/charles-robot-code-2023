package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ManualArm extends CommandBase {

  private ArmSubsystem armSubsystem;
  private DoubleSupplier armRotationSpeed;
  private DoubleSupplier armExtensionSpeed;

  public ManualArm(ArmSubsystem armSubsystem, DoubleSupplier armRotationSpeed, DoubleSupplier armExtensionSpeed) {
    this.armSubsystem = armSubsystem;
    this.armRotationSpeed = armRotationSpeed;
    this.armExtensionSpeed = armExtensionSpeed;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Math.abs(armRotationSpeed.getAsDouble()) > 0.1) {
      armSubsystem.setRotationSpeed(armRotationSpeed.getAsDouble());
    } else {
      armSubsystem.setRotationSpeed(0);
    }

    if (Math.abs(armExtensionSpeed.getAsDouble()) > 0.1) {
        armSubsystem.setExtensionSpeed(armExtensionSpeed.getAsDouble());
    } else {
      armSubsystem.setExtensionSpeed(0.025);
    }
  }


  @Override
  public void end(boolean interrupted) {
    // armSubsystem.setExtensionSpeed(0);
    // armSubsystem.setRotationSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
