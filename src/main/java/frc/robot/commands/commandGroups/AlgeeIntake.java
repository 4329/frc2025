package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.utilities.NotFinalSequentialCommandGroup;
import frc.robot.utilities.UnInstantCommand;

public class AlgeeIntake extends NotFinalSequentialCommandGroup {

    private ButtonRingController buttonRingController;

    public AlgeeIntake(
            Drivetrain drivetrain,
            ElevatorSubsystem elevatorSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            ButtonRingController buttonRingController) {

        this.buttonRingController = buttonRingController;

        addCommands(
                new ParallelCommandGroup(
                        new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                        new UnInstantCommand(
                                        () ->
                                                elevatorSubsystem.setSetpoint(
                                                        buttonRingController.getxOffset() < 0
                                                                ? ElevatorSubsystem.ElevatorPosition.ALGEE_HIGH
                                                                : ElevatorSubsystem.ElevatorPosition.ALGEE_LOW))
                                .until(elevatorSubsystem::atSetpoint)),
                new CenterOnAlgeeCommand(poseEstimationSubsystem, drivetrain, buttonRingController),
                new IntakeAlgeeCommand(algeeWheelSubsystem, 1));
    }

    @Override
    public void execute() {
        if (buttonRingController.getTagID() != 0) super.execute();
    }
}
