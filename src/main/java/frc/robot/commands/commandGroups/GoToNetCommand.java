package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;

public class GoToNetCommand extends LoggedParallelCommandGroup {

    public GoToNetCommand(
            AlgeePivotSubsystem algeePivotSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.NET));
    }
}
