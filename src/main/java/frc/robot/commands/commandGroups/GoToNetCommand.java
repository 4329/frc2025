package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class GoToNetCommand extends LoggedSequentialCommandGroup {

    public GoToNetCommand(
            AlgeePivotSubsystem algeePivotSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.NET),
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.NET)
        );
    }
}
