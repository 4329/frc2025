package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AlgeeScoreHigh extends LoggedSequentialCommandGroup {

    public AlgeeScoreHigh(
            ElevatorSubsystem elevatorSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem) {

        addCommands(
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ALGEE_HIGH),
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                new OuttakeAlgeeCommand(algeeWheelSubsystem));
    }
}
