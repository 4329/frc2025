package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AutoScoreCoralCommand extends LoggedSequentialCommandGroup {

    public AutoScoreCoralCommand(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ElevatorPosition elevatorPosition,
            DifferentialArmSubsystem differentialArmSubsystem) {

        addCommands(
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.ZERO),
                new ParallelCommandGroup(
                        new UnInstantCommand(
                                        "elevator",
                                        () ->
                                                elevatorSubsystem.setSetpoint(
                                                        switch (elevatorPosition) {
                                                            case L2 -> ElevatorSubsystem.ElevatorPosition.L2;
                                                            case L3 -> ElevatorSubsystem.ElevatorPosition.L3;
                                                            case L4 -> ElevatorSubsystem.ElevatorPosition.L4;
                                                            default -> ElevatorSubsystem.ElevatorPosition.L2;
                                                        }))
                                .whileLog(() -> !elevatorSubsystem.atSetpoint()),
                        new SetArmPitchCommand(
                                differentialArmSubsystem,
                                DifferentialArmSubsystem.DifferentialArmPitch.ONETHIRTYFIVE)));
    }
}
