package frc.robot.commands.elevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class SetElevatorCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ElevatorSubsystem.ElevatorPosition elevatorPosition;

    public SetElevatorCommand(
            ElevatorSubsystem elevatorSubsystem, ElevatorPosition elevatorPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setSetpoint(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint();
    }
}
