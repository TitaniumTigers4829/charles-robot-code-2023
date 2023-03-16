// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm.HoldArm;
import frc.robot.commands.arm.ManuallyControlArm;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.autonomous.FollowPathPlannerTrajectory;
import frc.robot.commands.autonomous.SimpleAuto;
import frc.robot.commands.autonomous.TwoConeBalanceAuto;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.claw.RunClaw;
import frc.robot.commands.claw.SetClawRotation;
import frc.robot.commands.drive.Balance;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemImpl;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystemImpl;
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
  public final ArmSubsystem armSubsystem;
  public final ClawSubsystem clawSubsystem;

  private final Joystick driverJoystick;
  private final Joystick operatorJoystick;

  public final Joystick buttonBoard, buttonBoard2;

  public RobotContainer() {
  
    driverJoystick = new Joystick(JoystickConstants.DRIVER_JOYSTICK_ID);
    operatorJoystick = new Joystick(JoystickConstants.OPERATOR_JOYSTICK_ID);

    buttonBoard = new Joystick(JoystickConstants.BUTTON_BOARD_ID);
    buttonBoard2 = new Joystick(JoystickConstants.BUTTON_BOARD_ID_2);

    driveSubsystem = new DriveSubsystemImpl();
    visionSubsystem = new VisionSubsystemImpl();
    armSubsystem = new ArmSubsystemImpl();
    clawSubsystem = new ClawSubsystemImpl();

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

  private static double modifyAxisCubed(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, 0.1);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_LEFT_STICK_X);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_LEFT_STICK_Y);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_RIGHT_STICK_X);
    JoystickButton driverRightBumper = new JoystickButton(driverJoystick, JoystickConstants.DRIVER_RIGHT_BUMPER_ID);

    Command driveCommand = new DriveCommand(driveSubsystem, visionSubsystem,
      () -> modifyAxisCubed(driverLeftStickY) * -1,
      () -> modifyAxisCubed(driverLeftStickX) * -1,
      () -> modifyAxisCubed(driverRightStickX) * -1,
      () -> !driverRightBumper.getAsBoolean()
    );

    driveSubsystem.setDefaultCommand(driveCommand);


    POVButton driverRightDirectionPad = new POVButton(driverJoystick, JoystickConstants.RIGHT_DPAD_ID);
    driverRightDirectionPad.onTrue(new InstantCommand(driveSubsystem::zeroHeading));
    driverRightDirectionPad.onTrue(new InstantCommand(driveSubsystem::zeroPitchAndRoll));

    
    /* Arm Buttons */
    DoubleSupplier operatorLeftStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.OPERATOR_LEFT_STICK_Y);
    DoubleSupplier operatorRightStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.OPERATOR_RIGHT_STICK_Y);

    Command manualArmCommand = new ManuallyControlArm(
      armSubsystem, 
      operatorLeftStickY, 
      operatorRightStickY
    );

    JoystickButton operatorYButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_Y_BUTTON_ID);
    operatorYButton.whileTrue(new HoldArm(armSubsystem));
    JoystickButton operatorAButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_A_BUTTON_ID);
    operatorAButton.whileTrue(new SetArmExtension(armSubsystem, .01));
    JoystickButton operatorXButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_X_BUTTON_ID);
    operatorXButton.onTrue(new InstantCommand(armSubsystem::resetExtensionEncoder));

    armSubsystem.setDefaultCommand(manualArmCommand);

    /* Claw Buttons */
//    JoystickButton operatorAButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_A_BUTTON_ID);
//    operatorAButton.onTrue(new OpenClaw(clawSubsystem));
//    JoystickButton operatorBButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_B_BUTTON_ID);
//    operatorBButton.onTrue(new CloseClaw(clawSubsystem));

    //Auto Place Buttons
    // JoystickButton autoplaceButton1 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_1);
    // autoplaceButton1.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton1.getAsBoolean(), 1));
    // JoystickButton autoplaceButton2 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_2);
    // autoplaceButton2.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton2.getAsBoolean(), 2));
    // JoystickButton autoplaceButton3 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_3);
    // autoplaceButton3.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton3.getAsBoolean(), 3));
    // JoystickButton autoplaceButton4 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_4);
    // autoplaceButton4.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton4.getAsBoolean(), 4));
    // JoystickButton autoplaceButton5 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_5);
    // autoplaceButton5.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton5.getAsBoolean(), 5));
    // JoystickButton autoplaceButton6 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_6);
    // autoplaceButton6.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton6.getAsBoolean(), 6));
    // JoystickButton autoplaceButton7 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_7);
    // autoplaceButton7.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton7.getAsBoolean(), 7));
    // JoystickButton autoplaceButton8 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_8);
    // autoplaceButton8.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton8.getAsBoolean(), 8));
    // JoystickButton autoplaceButton9 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_9);
    // autoplaceButton9.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton9.getAsBoolean(), 9));
    // JoystickButton autoplaceButton10 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_10);
    // autoplaceButton10.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton10.getAsBoolean(), 10));
    // JoystickButton autoplaceButton11 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_11);
    // autoplaceButton11.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton11.getAsBoolean(), 11));
    // JoystickButton autoplaceButton12 = new JoystickButton(buttonBoard, JoystickConstants.BUTTON_12);
    // autoplaceButton12.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton12.getAsBoolean(), 12));
    // POVButton autoplaceButton13 = new POVButton(buttonBoard, JoystickConstants.BUTTON_13);
    // autoplaceButton13.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton13.getAsBoolean(), 13));
    // POVButton autoplaceButton14 = new POVButton(buttonBoard, JoystickConstants.BUTTON_14);
    // autoplaceButton14.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton14.getAsBoolean(), 14));
    // POVButton autoplaceButton15 = new POVButton(buttonBoard, JoystickConstants.BUTTON_15);
    // autoplaceButton15.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton15.getAsBoolean(), 15));
    // POVButton autoplaceButton16 = new POVButton(buttonBoard, JoystickConstants.BUTTON_16);
    // autoplaceButton16.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton16.getAsBoolean(), 16));
    // JoystickButton autoplaceButton17 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_17);
    // autoplaceButton17.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton17.getAsBoolean(), 17));
    // JoystickButton autoplaceButton18 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_18);
    // autoplaceButton18.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton18.getAsBoolean(), 18));
    // JoystickButton autoplaceButton19 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_19);
    // autoplaceButton19.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton19.getAsBoolean(), 19));
    // JoystickButton autoplaceButton20 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_20);
    // autoplaceButton20.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton20.getAsBoolean(), 20));
    // JoystickButton autoplaceButton21 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_21);
    // autoplaceButton21.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton21.getAsBoolean(), 21));
    // JoystickButton autoplaceButton22 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_22);
    // autoplaceButton22.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton22.getAsBoolean(), 22));
    // JoystickButton autoplaceButton23 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_23);
    // autoplaceButton23.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton23.getAsBoolean(), 23));
    // JoystickButton autoplaceButton24 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_24);
    // autoplaceButton24.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton24.getAsBoolean(), 24));
    // JoystickButton autoplaceButton25 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_25);
    // autoplaceButton25.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton25.getAsBoolean(), 25));
    // JoystickButton autoplaceButton26 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_26);
    // autoplaceButton26.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton26.getAsBoolean(), 26));
    // JoystickButton autoplaceButton27 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_27);
    // autoplaceButton27.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, () -> !autoplaceButton27.getAsBoolean(), 27));
  }

  public Command getAutonomousCommand() {
    // return new TwoConeBalanceAuto(driveSubsystem, visionSubsystem);
    return new SimpleAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem);
  }
}