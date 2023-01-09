package frc.robot.commands.limb.arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimbSubsystem;


public class ExtendArm extends CommandBase {

    private LimbSubsystem limbSubsystem;

    public ExtendArm(LimbSubsystem limbSubsystem) {
        this.limbSubsystem = limbSubsystem;
        addRequirements(limbSubsystem);
    }

    @Override
    public void execute() {
        limbSubsystem.ExtendArm();
    }


    @Override
    public void end(boolean interrupted) {
        limbSubsystem.CloseArm();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

}
