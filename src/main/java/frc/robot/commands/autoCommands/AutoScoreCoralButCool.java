package frc.robot.commands.autoCommands;

import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AutoScoreCoralButCool extends LoggedSequentialCommandGroup {

    public AutoScoreCoralButCool(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ElevatorPosition elevatorPosition,
            ElevatorPosition scoreElevatorPosition,
            DifferentialArmSubsystem differentialArmSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            int num,
            boolean right) {

        addCommands(
                new LoggedParallelCommandGroup(
                        "Setup and minDist",
                        new CenterInAutoCommand(
                                poseEstimationSubsystem, drivetrain, num, right, CenterDistance.INITIAL),
                        new AutoPositionCoralCommand(
                                elevatorSubsystem,
                                differentialArmSubsystem,
                                elevatorPosition,
                                algeePivotSubsystem)),
                new CenterInAutoCommand(
                        poseEstimationSubsystem, drivetrain, num, right, CenterDistance.SCORING),
                new AutoActuallyScoreCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, scoreElevatorPosition),
                new DriveBackCommand(drivetrain));
    }
}
