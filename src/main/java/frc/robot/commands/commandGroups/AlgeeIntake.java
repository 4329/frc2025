package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;
import java.util.function.BooleanSupplier;

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

        BooleanSupplier condition =
                () -> buttonRingController.getLevel() != 0 && buttonRingController.getTagID() != 0;
        CenterOnTargetCommand center =
                new CenterOnTargetCommand(
                        buttonRingController::getTagID,
                        poseEstimationSubsystem,
                        drivetrain,
                        () -> 0.156,
                        CenterDistance.SCORING);

        addCommands(
                new UnInstantCommand("calcInitial", center::calcInitial),
                new LoggedParallelCommandGroup(
                                "PositionEverything",
                                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                                new UnInstantCommand(
                                                "SetElevatorByButtonRing",
                                                () ->
                                                        elevatorSubsystem.setSetpoint(
                                                                buttonRingController.getLevel() == 3
                                                                        ? ElevatorSubsystem.ElevatorPosition.ALGEE_HIGH
                                                                        : ElevatorSubsystem.ElevatorPosition.ALGEE_LOW))
                                        .whileLog(() -> !elevatorSubsystem.atSetpoint()))
                        .onlyIfLog(condition),
                new LoggedParallelCommandGroup(
                        "CenterIntake",
                        new IntakeAlgeeCommand(algeeWheelSubsystem),
                        center.onlyIfLog(condition)));
    }

    @Override
    public void initialize() {
        LEDState.algeeIntaking = true;
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        LEDState.algeeIntaking = false;
        super.end(interrupted);
    }
}
