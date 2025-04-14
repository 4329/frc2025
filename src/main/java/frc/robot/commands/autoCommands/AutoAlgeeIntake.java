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

    AlgeeWheelSubsystem algeeWheelSubsystem;

    public AutoAlgeeIntake(
            ElevatorSubsystem elevatorSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorPosition elevatorPosition) {

        this.algeeWheelSubsystem = algeeWheelSubsystem;

        addCommands(
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                new UnInstantCommand(
                        "intake algee",
                        () ->
                                elevatorSubsystem.setSetpoint(
                                        ElevatorPosition.L3.equals(elevatorPosition)
                                                ? ElevatorPosition.ALGEE_HIGH
                                                : ElevatorPosition.ALGEE_LOW)));
    }
}
