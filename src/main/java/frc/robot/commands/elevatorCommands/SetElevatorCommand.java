package frc.robot.commands.elevatorCommands;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class SetElevatorCommand extends LoggedCommandComposer {
    ElevatorSubsystem elevatorSubsystem;
    ElevatorSubsystem.ElevatorPosition elevatorPosition;
    boolean set;

    public SetElevatorCommand(
            ElevatorSubsystem elevatorSubsystem, ElevatorPosition elevatorPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        set = false;
    }

    @Override
    public void execute() {
        if (!set) {
            elevatorSubsystem.setSetpoint(elevatorPosition);
            set = true;
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint();
    }
}
