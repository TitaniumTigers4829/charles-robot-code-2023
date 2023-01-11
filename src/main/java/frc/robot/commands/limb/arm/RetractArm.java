package frc.robot.commands.limb.arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimbSubsystem;


public class RetractArm extends CommandBase {

    private LimbSubsystem limbSubsystem;

    public RetractArm(LimbSubsystem limbSubsystem) {
        this.limbSubsystem = limbSubsystem;
        addRequirements(limbSubsystem);
    }

    @Override
    public void execute() {
        limbSubsystem.CloseArm();
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

}
