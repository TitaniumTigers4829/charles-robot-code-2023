package frc.robot.commands.limb.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimbSubsystem;

import java.util.function.DoubleSupplier;


public class RotateWrist extends CommandBase {
    
    private double desiredRotation;
    private LimbSubsystem limbSubsystem;

    /** Creates a new RotateWrist command.
     * @param desiredRotation The desired rotation for the wrist. (In radians.)
     */
    public RotateWrist(DoubleSupplier desiredRotation, LimbSubsystem limbSubsystem) {
        this.desiredRotation = desiredRotation.getAsDouble();
        this.limbSubsystem = limbSubsystem;
        addRequirements(limbSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        limbSubsystem.SetDesiredWristRotation(0-9);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
