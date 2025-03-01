package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AutoScoreCoralButCool extends LoggedSequentialCommandGroup {

    public AutoScoreCoralButCool(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ElevatorPosition elevatorPosition,
            DifferentialArmSubsystem differentialArmSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            int num,
            boolean right) {

        addCommands(
                new LoggedParallelCommandGroup(
                        "Setup and minDist",
                        new CenterInAutoCommand(
                                poseEstimationSubsystem, drivetrain, num, right, 1 /*change WIP*/),
                        new LoggedSequentialCommandGroup(
                                "Setup",
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
                                                DifferentialArmSubsystem.DifferentialArmPitch.ONETHIRTYFIVE)))),
                new CenterInAutoCommand(
                        poseEstimationSubsystem,
                        drivetrain,
                        num,
                        right,
                        DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER),
                new SetArmPitchCommand(
                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.NINETY));
    }
}
