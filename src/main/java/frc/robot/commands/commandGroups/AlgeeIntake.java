package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;

public class AlgeeIntake extends LoggedParallelCommandGroup {

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
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                new UnInstantCommand(
                                "SetElevatorByButtonRing",
                                () ->
                                        elevatorSubsystem.setSetpoint(
                                                buttonRingController.getLevel() == 3
                                                        ? ElevatorSubsystem.ElevatorPosition.ALGEE_HIGH
                                                        : ElevatorSubsystem.ElevatorPosition.ALGEE_LOW))
                        .whileLog(() -> !elevatorSubsystem.atSetpoint()),
                new IntakeAlgeeCommand(algeeWheelSubsystem));
    }

    @Override
    public void execute() {
        if (buttonRingController.getLevel() != 0) super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // ugh
        algeeWheelSubsystem.stop();
        super.end(interrupted);
    }
}
