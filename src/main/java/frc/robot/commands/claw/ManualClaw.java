package frc.robot.commands.claw;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.extras.SmartDashboardLogger;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;

public class ManualClaw extends CommandBase {

  private ClawSubsystem clawSubsystem;
  private LEDSubsystem leds;
  private BooleanSupplier intakeCargo, expelCargo, rotateClaw180, isManual, switchMode, flipClawOtherWay;
  private double initialRotation;
  private boolean lastState;
  private boolean hasSetPos;
  private boolean lastSwitchMode;

  public ManualClaw(ClawSubsystem clawSubsystem, LEDSubsystem leds, BooleanSupplier intakeCargo, BooleanSupplier expelCargo, BooleanSupplier rotateClaw, BooleanSupplier isManual, BooleanSupplier switchMode, BooleanSupplier flipClawOtherWay) {
    this.clawSubsystem = clawSubsystem;
    this.leds = leds;
    this.intakeCargo = intakeCargo;
    this.expelCargo = expelCargo;
    this.rotateClaw180 = rotateClaw;
    this.isManual = isManual;
    this.switchMode = switchMode;
    this.flipClawOtherWay = flipClawOtherWay;
    addRequirements(clawSubsystem, leds);
  }

  @Override
  public void initialize() {
    initialRotation = clawSubsystem.getWristAngle();
    lastState = false;
    hasSetPos = false;
    lastSwitchMode = false;
  }

  @Override
  public void execute() {
    if (!NodeAndModeRegistry.isConeMode()) {
      clawSubsystem.open();    
    } else {
      clawSubsystem.close();
    }

    if (isManual.getAsBoolean()) {
      SmartDashboardLogger.infoBoolean("running", true);
      if (!NodeAndModeRegistry.isConeMode()) {
        if (intakeCargo.getAsBoolean()) {
          clawSubsystem.setIntakeSpeed(0.15);
        } else if (expelCargo.getAsBoolean()) {
          clawSubsystem.setIntakeSpeed(-0.15);
        } else {
          clawSubsystem.setIntakeSpeed(0.04);
        }
      } else {
        if (intakeCargo.getAsBoolean()) {
          clawSubsystem.close();
          clawSubsystem.setIntakeSpeed(0.25);
        } else if (expelCargo.getAsBoolean()) {
          clawSubsystem.setIntakeSpeed(0);
          clawSubsystem.open();
        } else {
          clawSubsystem.close();
          clawSubsystem.setIntakeSpeed(0);
        }
      }

      if (lastState != rotateClaw180.getAsBoolean()) {
          lastState = rotateClaw180.getAsBoolean();
          initialRotation = clawSubsystem.getWristAngle();
      }

      if (rotateClaw180.getAsBoolean()) {
        hasSetPos = false;
        if (initialRotation > 90) {
          clawSubsystem.setWristPosition(0);
        } else {
          clawSubsystem.setWristPosition(180);
        }
      } else {
        if (!hasSetPos) {
          clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
          hasSetPos = true;
        }
      }
      // if (flipClawOtherWay.getAsBoolean()) {
      //   hasSetPos = false;
      //   if (initialRotation > -90) {
      //     clawSubsystem.setWristPosition(0);
      //   } else {
      //     clawSubsystem.setWristPosition(-180);
      //   }
      // } else {
      //   if (!hasSetPos) {
      //     clawSubsystem.setWristPosition(clawSubsystem.getWristAngle());
      //     hasSetPos = true;
      //   }
      // }
    } else {
      SmartDashboardLogger.infoBoolean("running", false);
    }
    if (switchMode.getAsBoolean() && !lastSwitchMode) {
      // clawSubsystem.switchCargoMode();
      NodeAndModeRegistry.toggleMode();
      lastSwitchMode = true;
      if (NodeAndModeRegistry.isConeMode()) {
        leds.setProcess(LEDProcess.CONE);
      } else {
        leds.setProcess(LEDProcess.CUBE);
      }
    } else if (!switchMode.getAsBoolean()) {
      lastSwitchMode = false;
    }
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
