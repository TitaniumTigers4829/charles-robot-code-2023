// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemImpl;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemImpl;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemImpl;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ArmSubsystem armSubsystem;

  private final Joystick driverJoystick;

  private final JoystickButton rightBumper, aButton;

  public final Joystick buttonBoard;

  private final InstantCommand enableLock;
  private final InstantCommand disableLock;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
  
    driverJoystick = new Joystick(JoystickConstants.DRIVER_JOYSTICK_ID);
    rightBumper = new JoystickButton(driverJoystick, JoystickConstants.RIGHT_BUMPER_ID);
    aButton = new JoystickButton(driverJoystick, JoystickConstants.A_BUTTON_ID);

    buttonBoard = new Joystick(JoystickConstants.BUTTON_BOARD_ID);

    driveSubsystem = new DriveSubsystemImpl();
    visionSubsystem = new VisionSubsystemImpl();
    armSubsystem = new ArmSubsystemImpl();

    DoubleSupplier leftStickX = () -> driverJoystick.getRawAxis(JoystickConstants.LEFT_STICK_X);
    DoubleSupplier leftStickY = () -> driverJoystick.getRawAxis(JoystickConstants.LEFT_STICK_Y);
    DoubleSupplier rightStickX = () -> driverJoystick.getRawAxis(JoystickConstants.RIGHT_STICK_X);

    

    // Command driveCommand = new DriveCommand(driveSubsystem, visionSubsystem,
    //   () -> modifyAxisSquared(leftStickY) * -1, 
    //   () -> modifyAxisSquared(leftStickX) * -1, 
    //   () -> modifyAxisSquared(rightStickX) * -1, 
    //   () -> !rightBumper.getAsBoolean()
    // );

    enableLock = new InstantCommand(armSubsystem::lockExtensionSolenoid, armSubsystem);
    disableLock = new InstantCommand(armSubsystem::unlockExtensionSolenoid, armSubsystem);

    JoystickButton enableButton = new JoystickButton(driverJoystick, JoystickConstants.A_BUTTON_ID);
    JoystickButton disableButton = new JoystickButton(driverJoystick, JoystickConstants.B_BUTTON_ID);

    enableButton.onTrue(enableLock);
    disableButton.onTrue(disableLock);

    // driveSubsystem.setDefaultCommand(driveCommand);
    new JoystickButton(driverJoystick, JoystickConstants.A_BUTTON_ID).whileTrue(new RunCommand(() -> armSubsystem.manuallyRotate(leftStickY)));

    configureButtonBindings();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxisSquared(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, 0.1);

    // Cube the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    POVButton rightDirectionPad = new POVButton(driverJoystick, JoystickConstants.RIGHT_DPAD_ID);
    rightDirectionPad.onTrue(new InstantCommand(driveSubsystem::zeroHeading));

    // JoystickButton bButton = new JoystickButton(driverJoystick, JoystickConstants.B_BUTTON_ID);
    // bButton.whileTrue(new FollowRealTimeTrajectory(driveSubsystem, () -> !bButton.getAsBoolean()));

    // new JoystickButton(driverJoystick, JoystickConstants.A_BUTTON_ID).onTrue(new InstantCommand(armSubsystem::lockExtensionSolenoid, armSubsystem));
    // new JoystickButton(driverJoystick, JoystickConstants.B_BUTTON_ID).onTrue(new InstantCommand(armSubsystem::unlockExtensionSolenoid, armSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}