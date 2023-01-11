// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.limb.claw.ToggleClaw;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public ArmSubsystem armSubsystem;
  public ElevatorSubsystem elevatorSubsystem;

  public Joystick buttonBoard;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem = new ArmSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    buttonBoard = new Joystick(JoystickConstants.buttonBoardID);
    JoystickButton clawButton = new JoystickButton(buttonBoard, JoystickConstants.clawButtonID);
    clawButton.whileTrue(new ToggleClaw(armSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}