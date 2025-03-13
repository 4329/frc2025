package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.driveCommands.CenterOnAlgeeCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AlgeeIntake extends LoggedSequentialCommandGroup {

    private ButtonRingController buttonRingController;
    AlgeeWheelSubsystem algeeWheelSubsystem;

    public AlgeeIntake(
            Drivetrain drivetrain,
            ElevatorSubsystem elevatorSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            ButtonRingController buttonRingController) {

        this.buttonRingController = buttonRingController;
        this.algeeWheelSubsystem = algeeWheelSubsystem;

        addCommands(
                new LoggedParallelCommandGroup(
                        "SetInitialPositions",
                        new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                        new UnInstantCommand(
                                        "SetElevatorByButtonRing",
                                        () ->
                                                elevatorSubsystem.setSetpoint(
                                                        buttonRingController.getLevel() == 3
                                                                ? ElevatorSubsystem.ElevatorPosition.ALGEE_HIGH
                                                                : ElevatorSubsystem.ElevatorPosition.ALGEE_LOW))
                                .whileLog(() -> !elevatorSubsystem.atSetpoint())
                ),

                new CenterOnAlgeeCommand(
                        poseEstimationSubsystem, drivetrain, buttonRingController, CenterDistance.SCORING),
                new IntakeAlgeeCommand(algeeWheelSubsystem)
        );
    }

    @Override
    public void execute() {
        if (buttonRingController.getTagID() != 0) super.execute();
    }

}
