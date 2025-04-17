package frc.robot.commands.autoCommands;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AutoAlgeeIntake extends LoggedSequentialCommandGroup {

    public AutoAlgeeIntake(
            ElevatorSubsystem elevatorSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorPosition elevatorPosition) {

        addCommands(
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                new UnInstantCommand(
                        "intake algee",
                        () -> elevatorSubsystem.setSetpoint(elevatorPosition)));
    }
}
