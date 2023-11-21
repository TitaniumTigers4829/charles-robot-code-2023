package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ManualArm;
import frc.robot.commands.arm.MoveArmToStowed;
import frc.robot.commands.arm.PickupGamePiece;
import frc.robot.commands.autonomous.AutoPickupLoadingStation;
import frc.robot.commands.autonomous.AutoPlace;
import frc.robot.commands.autonomous.SimpleAuto;
import frc.robot.commands.autonomous.TwoPieceBalanceBlueAuto;
import frc.robot.commands.autonomous.TwoPieceBalanceRedAuto;
import frc.robot.commands.autonomous.TwoPieceBlueAuto;
import frc.robot.commands.autonomous.TwoPieceRedAuto;
import frc.robot.commands.autonomous.simple.DriveForwardThenDriveBackward;
import frc.robot.commands.autonomous.simple.JustDriveForward;
import frc.robot.commands.autonomous.simple.PlaceConeHighAuto;
import frc.robot.commands.autonomous.simple.PlaceCubeHighAuto;
import frc.robot.commands.claw.ManualClaw;
import frc.robot.commands.arm.PlaceGamePiece;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.NodeAndModeRegistry;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemImpl;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystemImpl;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemImpl;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.leds.LEDSubsystemImplSpark;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemImpl;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final LEDSubsystem leds;

  private final Joystick driverJoystick;
  private final Joystick operatorJoystick;
  private final Joystick buttonBoard1;
  private final Joystick buttonBoard2;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    driverJoystick = new Joystick(JoystickConstants.DRIVER_JOYSTICK_ID);
    operatorJoystick = new Joystick(JoystickConstants.OPERATOR_JOYSTICK_ID);

    buttonBoard1 = new Joystick(JoystickConstants.BUTTON_BOARD_1_ID);
    buttonBoard2 = new Joystick(JoystickConstants.BUTTON_BOARD_2_ID);

    driveSubsystem = new DriveSubsystemImpl();
    visionSubsystem = new VisionSubsystemImpl();
    armSubsystem = new ArmSubsystemImpl();
    clawSubsystem = new ClawSubsystemImpl();
    leds = new LEDSubsystemImplSpark();

    autoChooser = new SendableChooser<Command>();

    Command twoPieceBalanceAuto = DriverStation.getAlliance() == Alliance.Blue 
      ? new TwoPieceBalanceBlueAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem, leds)
      : new TwoPieceBalanceRedAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem, leds);

    Command twoPieceNoBalanceAuto = DriverStation.getAlliance() == Alliance.Blue 
      ? new TwoPieceBlueAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem, leds)
      : new TwoPieceRedAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem, leds);

    autoChooser.setDefaultOption("Simple Auto", new SimpleAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem));
    autoChooser.addOption("2 piece balance", twoPieceBalanceAuto);
    autoChooser.addOption("2 piece no balance", twoPieceNoBalanceAuto);
    autoChooser.addOption("Place cone", new PlaceConeHighAuto(armSubsystem, clawSubsystem));
    autoChooser.addOption("Place cube", new PlaceCubeHighAuto(armSubsystem, clawSubsystem));
    autoChooser.addOption("Drive forward then back", new DriveForwardThenDriveBackward(driveSubsystem, visionSubsystem));
    autoChooser.addOption("Just Drive Forward", new JustDriveForward(driveSubsystem));
    autoChooser.addOption("No Auto", null);
    SmartDashboard.putData("Auto chooser", autoChooser);
    
    driveSubsystem.zeroHeading();
    driveSubsystem.zeroPitchAndRoll();
    driveSubsystem.resetOdometry(new Pose2d());

    leds.setProcess(LEDProcess.DEFAULT);
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

  private static double modifyAxisCubed(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, 0.1);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  private static double[] modifyAxisCubedPolar(DoubleSupplier xJoystick, DoubleSupplier yJoystick) {
    double xInput = deadband(xJoystick.getAsDouble(), 0.1);
    double yInput = deadband(yJoystick.getAsDouble(), 0.1);
    if (Math.abs(xInput) > 0 && Math.abs(yInput) > 0) {
      double theta = Math.atan(xInput / yInput);
      double hypotenuse = Math.sqrt(xInput * xInput + yInput * yInput);
      double cubedHypotenuse = Math.pow(hypotenuse, 3);
      xInput = Math.copySign(Math.sin(theta) * cubedHypotenuse, xInput);
      yInput = Math.copySign(Math.cos(theta) * cubedHypotenuse, yInput);
      return new double[]{xInput, yInput};
    }
    return new double[]{ Math.copySign(xInput * xInput * xInput, xInput),  Math.copySign(yInput * yInput * yInput, yInput)};
  }

  public void teleopInit() {
    clawSubsystem.setWristPosition(0);
    armSubsystem.setRotationSpeed(0);
    armSubsystem.setExtensionSpeed(0);
    NodeAndModeRegistry.init();
    SmarterDashboardRegistry.updateAlliance();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Drive Buttons */
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_LEFT_STICK_X);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_LEFT_STICK_Y);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_RIGHT_STICK_X);
    JoystickButton driverRightBumper = new JoystickButton(driverJoystick, JoystickConstants.DRIVER_RIGHT_BUMPER_ID);

    Command driveCommand = new DriveCommand(driveSubsystem, visionSubsystem,
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[0] * -1,
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[1] * -1,
      () -> modifyAxisCubed(driverRightStickX) * -1,
      () -> !driverRightBumper.getAsBoolean()
    );

    driveSubsystem.setDefaultCommand(driveCommand);

    POVButton driverRightDirectionPad = new POVButton(driverJoystick, JoystickConstants.RIGHT_DPAD_ID);
    driverRightDirectionPad.onTrue(new InstantCommand(driveSubsystem::zeroHeading));
    driverRightDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometryAndRotation(driveSubsystem.getPose(), driveSubsystem.getHeading())));
    driverRightDirectionPad.onTrue(new InstantCommand(driveSubsystem::zeroPitchAndRoll));

    JoystickButton driverBButton = new JoystickButton(driverJoystick, JoystickConstants.DRIVER_B_BUTTON_ID);
    driverBButton.whileTrue(new AutoPlace(driveSubsystem, visionSubsystem, leds, () -> !driverBButton.getAsBoolean()));

    JoystickButton driverAButton = new JoystickButton(driverJoystick, JoystickConstants.DRIVER_A_BUTTON_ID);
    driverAButton.whileTrue(new AutoPickupLoadingStation(driveSubsystem, visionSubsystem, leds, () -> !driverAButton.getAsBoolean()));


    /* Arm Buttons */
    DoubleSupplier operatorLeftStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.OPERATOR_LEFT_STICK_Y);
    DoubleSupplier operatorRightStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.OPERATOR_RIGHT_STICK_Y);
    BooleanSupplier operatorRightTrigger = () -> (operatorJoystick.getRawAxis(3) > 0.33);
    BooleanSupplier operatorLeftTrigger = () -> (operatorJoystick.getRawAxis(2) > 0.33);

    JoystickButton operatorXButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_X_BUTTON_ID);
    JoystickButton operatorYButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_Y_BUTTON_ID);
    JoystickButton operatorRightBumper = new JoystickButton(operatorJoystick, JoystickConstants.DRIVER_RIGHT_BUMPER_ID);
    
    // Command manualArmCommand = new ManualArm(
    //   armSubsystem, 
    //   operatorLeftStickY, 
    //   operatorRightStickY,
    //   operatorRightBumper::getAsBoolean
    // );
    
  //   armSubsystem.setDefaultCommand(manualArmCommand);

  //   DoubleSupplier yAxis = () -> buttonBoard1.getRawAxis(1);
  //   BooleanSupplier isBlueButtonPressed = () -> (yAxis.getAsDouble() > 0.2);

  //   Command manualClawCommand = new ManualClaw(
  //     clawSubsystem, 
  //     leds,
  //     operatorXButton::getAsBoolean,
  //     operatorYButton::getAsBoolean,
  //     operatorRightTrigger,
  //     operatorRightBumper::getAsBoolean,
  //     isBlueButtonPressed,
  //     operatorLeftTrigger
  //   );
    
  //   clawSubsystem.setDefaultCommand(manualClawCommand);
  
  //   /* Automation Buttons */
  //   // // JoystickButton operatorAButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_A_BUTTON_ID);
  //   // operatorAButton.whileTrue(new PlaceGamePiece(armSubsystem, clawSubsystem));
  //   // // TODO: Test
  //   // operatorAButton.onFalse(new MoveArmToStowed(armSubsystem, clawSubsystem).beforeStarting(new WaitCommand(.7)));

  //   // JoystickButton operatorBButton = new JoystickButton(operatorJoystick, JoystickConstants.OPERATOR_B_BUTTON_ID);
  //   // operatorBButton.whileTrue(new PickupGamePiece(armSubsystem, clawSubsystem, ArmConstants.PICKUP_LOADING_STATION_ROTATION, ArmConstants.PICKUP_LOADING_STATION_EXTENSION));
  //   // operatorBButton.onFalse(new MoveArmToStowed(armSubsystem, clawSubsystem));

  //   POVButton operatorRightDirectionPad = new POVButton(operatorJoystick, 90);
  //   operatorRightDirectionPad.onTrue(new InstantCommand(armSubsystem::resetExtensionEncoder));
  //   operatorRightDirectionPad.onTrue(new InstantCommand(clawSubsystem::zeroWristEncoder));

  //   /* Button Board Buttons */
  //   DoubleSupplier xAxis = () -> buttonBoard1.getRawAxis(0);
  //   DoubleSupplier zAxis = () -> buttonBoard1.getRawAxis(2);
  //   DoubleSupplier zAxis2 = () -> buttonBoard2.getRawAxis(2);

  //   BooleanSupplier isRedButtonPressed = () -> (yAxis.getAsDouble() < -0.2);
  //   // Trigger onRedButtonPressed = new Trigger(isRedButtonPressed);

  //   BooleanSupplier isBigButton1Pressed = () -> (zAxis.getAsDouble() < -0.2);
  //   Trigger onBigButton1Pressed = new Trigger(isBigButton1Pressed);
  //   onBigButton1Pressed.whileTrue(new PlaceGamePiece(armSubsystem, clawSubsystem, isRedButtonPressed));
  //   onBigButton1Pressed.onFalse(new MoveArmToStowed(armSubsystem, clawSubsystem).beforeStarting(new WaitCommand(.7)));

  //   BooleanSupplier isBigButton2Pressed = () -> (zAxis.getAsDouble() > 0.2);
  //   Trigger onBigButton2Pressed = new Trigger(isBigButton2Pressed);
  //   onBigButton2Pressed.whileTrue(new PickupGamePiece(armSubsystem, clawSubsystem, leds, ArmConstants.PICKUP_GROUND_ROTATION, ArmConstants.PICKUP_GROUND_EXTENSION, false));
  //   onBigButton2Pressed.onFalse(new MoveArmToStowed(armSubsystem, clawSubsystem));
    
  //   BooleanSupplier isBigButton3Pressed = () -> (xAxis.getAsDouble() < -0.2);
  //   Trigger onBigButton3Pressed = new Trigger(isBigButton3Pressed);
  //   onBigButton3Pressed.whileTrue(new PickupGamePiece(armSubsystem, clawSubsystem, leds, ArmConstants.PICKUP_CHUTE_ROTATION, ArmConstants.PICKUP_CHUTE_EXTENSION, false));
  //   onBigButton3Pressed.onFalse(new MoveArmToStowed(armSubsystem, clawSubsystem));

  //   BooleanSupplier isBigButton4Pressed = () -> (xAxis.getAsDouble() > 0.2);
  //   Trigger onBigButton4Pressed = new Trigger(isBigButton4Pressed);
  //   onBigButton4Pressed.whileTrue(new PickupGamePiece(armSubsystem, clawSubsystem, leds, ArmConstants.PICKUP_LOADING_STATION_ROTATION, ArmConstants.PICKUP_LOADING_STATION_EXTENSION, true));
  //   onBigButton4Pressed.onFalse(new MoveArmToStowed(armSubsystem, clawSubsystem));
 
  //   /* Auto Place Buttons */
  //   JoystickButton autoplaceButton1 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_1);
  //   autoplaceButton1.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(1)));
  //   JoystickButton autoplaceButton2 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_2);
  //   autoplaceButton2.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(2)));
  //   JoystickButton autoplaceButton3 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_3);
  //   autoplaceButton3.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(3)));
  //   JoystickButton autoplaceButton4 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_4);
  //   autoplaceButton4.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(4)));
  //   JoystickButton autoplaceButton5 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_5);
  //   autoplaceButton5.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(5)));
  //   JoystickButton autoplaceButton6 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_6);
  //   autoplaceButton6.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(6)));
  //   JoystickButton autoplaceButton7 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_7);
  //   autoplaceButton7.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(7)));
  //   JoystickButton autoplaceButton8 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_8);
  //   autoplaceButton8.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(8)));
  //   JoystickButton autoplaceButton9 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_9);
  //   autoplaceButton9.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(9)));
  //   JoystickButton autoplaceButton10 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_10);
  //   autoplaceButton10.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(10)));
  //   JoystickButton autoplaceButton11 = new JoystickButton(buttonBoard1, JoystickConstants.BUTTON_11);
  //   autoplaceButton11.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(11)));
  //   Trigger autoplaceButton12 = new Trigger(() -> (zAxis2.getAsDouble() > 0));
  //   autoplaceButton12.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(12)));
  //   POVButton autoplaceButton13 = new POVButton(buttonBoard1, JoystickConstants.BUTTON_13);
  //   autoplaceButton13.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(13)));
  //   POVButton autoplaceButton14 = new POVButton(buttonBoard1, JoystickConstants.BUTTON_14);
  //   autoplaceButton14.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(14)));
  //   POVButton autoplaceButton15 = new POVButton(buttonBoard1, JoystickConstants.BUTTON_15);
  //   autoplaceButton15.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(15)));
  //   POVButton autoplaceButton16 = new POVButton(buttonBoard1, JoystickConstants.BUTTON_16);
  //   autoplaceButton16.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(16)));

  //   JoystickButton autoplaceButton17 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_17);
  //   autoplaceButton17.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(17)));
  //   JoystickButton autoplaceButton18 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_18);
  //   autoplaceButton18.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(18)));
  //   JoystickButton autoplaceButton19 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_19);
  //   autoplaceButton19.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(19)));
  //   JoystickButton autoplaceButton20 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_20);
  //   autoplaceButton20.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(20)));
  //   JoystickButton autoplaceButton21 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_21);
  //   autoplaceButton21.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(21)));
  //   JoystickButton autoplaceButton22 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_22);
  //   autoplaceButton22.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(22)));
  //   JoystickButton autoplaceButton23 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_23);
  //   autoplaceButton23.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(23)));
  //   JoystickButton autoplaceButton24 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_24);
  //   autoplaceButton24.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(24)));
  //   JoystickButton autoplaceButton25 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_25);
  //   autoplaceButton25.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(25)));
  //   JoystickButton autoplaceButton26 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_26);
  //   autoplaceButton26.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(26)));
  //   JoystickButton autoplaceButton27 = new JoystickButton(buttonBoard2, JoystickConstants.BUTTON_27);
  //   autoplaceButton27.onTrue(new InstantCommand(() -> NodeAndModeRegistry.setSelectedNode(27)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return new TwoConeBalanceAuto(driveSubsystem, visionSubsystem, armSubsystem, clawSubsystem);
    // return null;
  }
}