package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimbSubsystem;

import java.util.function.DoubleSupplier;


public class GoToHeight extends CommandBase {

    private final double desiredPosition;
    private final ElevatorSubsystem elevatorSubsystem;

    public GoToHeight(DoubleSupplier desiredPosition, ElevatorSubsystem elevatorSubsystem) {
        this.desiredPosition = desiredPosition.getAsDouble();
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements();
    }


    @Override
    public void execute() {
        // elevatorSubsystem.setDesiredElevatorHeight(desiredPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
